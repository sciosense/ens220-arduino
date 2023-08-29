#include <Arduino.h>
#include <ens220.h>
#include "i2c_interface.h"

using namespace ScioSense;

// Define the I2C address of the ENS220
#define I2C_ADDRESS 0x20

// Define the speed of the Serial communication with the computer
#define SERIAL_BAUDRATE 57600

// If the following line is un-commented, debug messages will be printed through the Serial port
//#define DEBUG_ENS220

// Create an ENS220 sensor object
ENS220 ens220;

// Create an I2C interface
I2cInterface i2c_1;

void SingleShotMeasure_setup()
{
    Serial.println("Starting ENS220 example 01_Basic_I2C_SingleShot");
    
    // Start the communication, confirm the device PART_ID, and read the device UID
    i2c_1.begin(Wire, I2C_ADDRESS);
    
    while(ens220.begin(&i2c_1) != true)
      {
        Serial.println("Waiting for I2C to start");
        delay(1000);
      }
    
    Serial.print("Device UID: "); Serial.println(ens220.getUID(), HEX);

    // Choose the desired configuration of the sensor. In this example we will use the Lowest Noise settings from the datasheet
    ens220.setDefaultConfiguration();
    // Set the Pressure ADC conversion time (MEAS_CFG register, field P_CONV)
    ens220.setPressureConversionTime(ENS220::PressureConversionTime::T_16_4);
    // Set the Oversampling of pressure measurements (OVS_CFG register, field OVSP) 
    ens220.setOversamplingOfPressure(ENS220::Oversampling::N_128);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220::Oversampling::N_128);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220::PressureTemperatureRatio::PT_1);
    // Set the operation to One shot (STBY_CFG register, field STBY_T)
    ens220.setStandbyTime(ENS220::StandbyTime::OneShotOperation);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220::PressureDataPath::Direct);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();
}

void SingleShotMeasure_loop()
{
    // Start single shot measurement
    ens220.singleShotMeasure(ENS220::Sensor::TemperatureAndPressure);
    
    // Wait until the measurement is ready
    ens220.waitSingleShot();
    
    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result = ens220.update();   
     
    if(result == ENS220::Result::Ok)
    {
      if(hasFlag(ens220.getDataStatus(), ENS220::DataStatus::PressureReady) && hasFlag(ens220.getDataStatus(), ENS220::DataStatus::TemperatureReady))
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