#include <Arduino.h>
#include <ens220.h>
#include "i2c_interface_soft.h"

// Define the I2C address of the ENS220
#define I2C_ADDRESS 0x20

// Define the speed of the Serial communication with the computer
#define SERIAL_BAUDRATE 57600

// Define the pins for the interfaces
#define SDA_PIN_1 21
#define SCL_PIN_1 22
#define SDA_PIN_2 18
#define SCL_PIN_2 19

// If the following line is un-commented, debug messages will be printed through the Serial port
//#define DEBUG_ENS220

// Create two ENS220 sensor objects
ENS220 ens220_1;
ENS220 ens220_2;

// Create two I2C interfaces
ScioSense_Arduino_SlowSoftI2CMaster_Config i2c_1;
ScioSense_Arduino_SlowSoftI2CMaster_Config i2c_2;

// Variable that stores the initial offset
float initial_offset = 0;

void HeightDifference_setup(ENS220& ens220)
{
    Serial.println("Starting ENS220 example 01_Basic_I2C_SingleShot");

    // Start the communication, confirm the device PART_ID, and read the device UID

    while(ens220.init() != true)
      {
        Serial.println("Waiting for I2C to start");
        delay(1000);
      }

    Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);

    // Choose the desired configuration of the sensor. In this example we will use the Lowest Noise settings from the datasheet
    ens220.setDefaultConfiguration();
    // Set the Pressure ADC conversion time (MEAS_CFG register, field P_CONV)
    ens220.setPressureConversionTime(ENS220_PRESSURE_CONVERSION_TIME_T_16_4);
    // Set the Oversampling of pressure measurements (OVS_CFG register, field OVSP)
    ens220.setOversamplingOfPressure(ENS220_OVERSAMPLING_N_32);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220_OVERSAMPLING_N_8);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220_PRESSURE_TEMPERATURE_RATIO_PT_1);
    // Set the operation to One shot (STBY_CFG register, field STBY_T)
    ens220.setStandbyTime(ENS220_STANDBY_TIME_ONE_SHOT_OPERATION);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220_PRESSURE_DATA_PATH_DIRECT);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();

}

void HeightDifference_zero(ENS220& ens220_1, ENS220& ens220_2)
{
    // Start single shot measurement on both devices
    ens220_1.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);
    ens220_2.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);

    // Wait until the measurement is ready
    ens220_2.waitSingleShot();

    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result_1 = ens220_1.update();
    auto result_2 = ens220_2.update();

    if(result_1 == RESULT_OK && result_2 == RESULT_OK)
    {
      float height_difference = ens220_1.CalculateHeightDifference(ens220_2);
      {
          // Take the initial offset to remove it from future measurements
          initial_offset = height_difference;
      }
    }
}

void HeightDifference_loop(ENS220& ens220_1, ENS220& ens220_2)
{
    // Start single shot measurement on both devices
    ens220_1.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);
    ens220_2.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);

    // Wait until the measurement is ready
    ens220_2.waitSingleShot();

    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result_1 = ens220_1.update();
    auto result_2 = ens220_2.update();

    if(result_1 == RESULT_OK && result_2 == RESULT_OK)
    {
      float height_difference = ens220_1.CalculateHeightDifference(ens220_2);
      {
          // Send the values that were collected during the ens220.update()
          Serial.print("H[m]:");
          Serial.println(height_difference - initial_offset);
      }
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    #ifdef DEBUG_ENS220
        ens220.enableDebugging(Serial);
    #endif

    ScioSense_Arduino_SlowSoftI2CMaster_begin(ens220_1, i2c_1, I2C_ADDRESS, SDA_PIN_1, SCL_PIN_1);
    HeightDifference_setup(ens220_1);

    ScioSense_Arduino_SlowSoftI2CMaster_begin(ens220_1, i2c_1, I2C_ADDRESS, SDA_PIN_2, SCL_PIN_2);
    HeightDifference_setup(ens220_2);

    HeightDifference_zero(ens220_1, ens220_2);
}

void loop()
{
    HeightDifference_loop(ens220_1, ens220_2);
    delay(200);
}