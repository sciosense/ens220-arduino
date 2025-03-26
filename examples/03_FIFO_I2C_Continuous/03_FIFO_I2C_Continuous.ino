#include <Arduino.h>
#include <ens220.h>

// Define the I2C address of the ENS220
#define I2C_ADDRESS 0x20

// Define the speed of the Serial communication with the computer
#define SERIAL_BAUDRATE 57600

// Define the Interrupt pin to be used
#define INTN_1 2

// If the following line is un-commented, debug messages will be printed through the Serial port
//#define DEBUG_ENS220

// Create an ENS220 sensor object
ENS220 ens220;

void ContinuousModeWithFIFO_setup()
{
    Serial.println("Starting ENS220 example 03_FIFO_I2C_Continuous");

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
    ens220.setOversamplingOfPressure(ENS220_OVERSAMPLING_N_16);
    // Set the Oversampling of temperature measurements (OVS_CFG register, field OVST)
    ens220.setOversamplingOfTemperature(ENS220_OVERSAMPLING_N_16);
    // Set the ratio between P and T measurements as produced by the measurement engine (MEAS_CFG register, field PT_RATE)
    ens220.setPressureTemperatureRatio(ENS220_PRESSURE_TEMPERATURE_RATIO_PT_32);
    // Set whether to use the FIFO buffer, a moving average, or none (MODE_CFG register, field FIFO_MODE)
    ens220.setPressureDataPath(ENS220_PRESSURE_DATA_PATH_FIFO);
    // Enable interrupts (INTF_CFG register, field INT_EN)
    ens220.setInterfaceConfiguration(ENS220_INTERFACE_CONFIGURATION_INTERRUPT_ENABLE);
    // Choose events that generate interrupts (INT_CFG register)
    ens220.setInterruptConfiguration(ENS220_INTERRUPT_CONFIGURATION_TEMPERATURE_DATA_READY | ENS220_INTERRUPT_CONFIGURATION_PRESSURE_FIFO_FULL);

    // Set which pin to use to poll for interrupts
    ens220.setInterruptPin(INTN_1);

    // Write the desired configuration into the sensor
    ens220.writeConfiguration();

    // Start continuous measurements
    ens220.startContinuousMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);
}

void ContinuousModeWithFIFO_loop()
{
    // Poll the interrupt pin until a new value is available
    ens220.waitInterrupt();

    // Check the DATA_STAT from the sensor. If data is available, it reads it
    auto result= ens220.update();
    if(result == RESULT_OK)
    {
      if(ens220.hasInterruptStatusFlag(ENS220_INTERRUPT_STATUS_FIFO_FULL))
      {
          for(int i=0; i<32; i++)
          {
              // Send the pressure value that was collected during the ens220.update()
              Serial.print("P[hPa]:");
              Serial.println(ens220.getPressureHectoPascal(i));
          }
      }

      if(ens220.hasInterruptStatusFlag(ENS220_INTERRUPT_STATUS_TEMPERATURE))
      {
          // Send the temperature value that was collected during the ens220.update()
          Serial.print("T[C]:");
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

    ContinuousModeWithFIFO_setup();
}

void loop()
{
    ContinuousModeWithFIFO_loop();
}