#include <Arduino.h>
#include <ens220.h>

// Define the I2C address of the ENS220
#define I2C_ADDRESS 0x20

// Define the speed of the Serial communication with the computer
#define SERIAL_BAUDRATE 57600

// If the following line is un-commented, debug messages will be printed through the Serial port
//#define DEBUG_ENS220

// Create an ENS220 sensor object
ENS220 ens220;

void SingleShotMeasure_setup()
{
    Serial.println("Starting ENS220 example 01_Basic_I2C_SingleShot");

    // Start the communication, confirm the device PART_ID, and read the device UID
    ens220.begin(&Wire, I2C_ADDRESS);

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
    ens220.setOversamplingOfPressure(ENS220_OVERSAMPLING_N_128);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220_OVERSAMPLING_N_128);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220_PRESSURE_TEMPERATURE_RATIO_PT_1);
    // Set the operation to One shot (STBY_CFG register, field STBY_T)
    ens220.setStandbyTime(ENS220_STANDBY_TIME_ONE_SHOT_OPERATION);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220_PRESSURE_DATA_PATH_DIRECT);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();
}

void SingleShotMeasure_loop()
{
    // Start single shot measurement
    ens220.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);

    // Wait until the measurement is ready
    ens220.waitSingleShot();

    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result = ens220.update();

    if(result == RESULT_OK)
    {
      if(ens220.hasDataStatusFlag(ENS220_DATA_STATUS_PRESSURE_READY) && ens220.hasDataStatusFlag(ENS220_DATA_STATUS_TEMPERATURE_READY))
      {
          // Send the values that were collected during the ens220.update()
          Serial.print("P[hPa]:");
          Serial.print(ens220.getPressureHectoPascal());
          Serial.print("\tT[C]:");
          Serial.println(ens220.getTempCelsius());
      }
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    Wire.begin();

    #ifdef DEBUG_ENS220
        ens220.enableDebugging(Serial);
    #endif

    SingleShotMeasure_setup();
}

void loop()
{
    SingleShotMeasure_loop();
}