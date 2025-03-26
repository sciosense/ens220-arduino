#ifndef SCIOSENSE_ENS220_H
#define SCIOSENSE_ENS220_H

#include <stdint.h>

#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>

#include "lib/ens220/ScioSense_Ens220.h"
#include "lib/io/ScioSense_IOInterface_Arduino_I2C.h"
#include "lib/io/ScioSense_IOInterface_Arduino_Ens220_SPI.h"

#ifndef NOT_A_PIN
#define NOT_A_PIN 0
#endif

class ENS220 : public ScioSense_Ens220
{
public:
    ENS220();
    ~ENS220();

public:
    inline void begin(TwoWire* wire, const uint8_t address = 0x20);                                 // Connnects to ENS220 using the given TwoWire object and address
    inline void begin(SPIClass* spi, const uint8_t chipSelect, const bool spi3WireMode = false, const SPISettings settings= SPISettings(2000000, MSBFIRST, SPI_MODE0));  // Connnects to ENS220 using arduinos SPI class
    inline bool init();                                                                             // Resets the device to IDLE and reads PART_ID and UID
    bool        isConnected();                                                                      // checks if the read PART_ID matches the expected value; returns true, if so.

public:
    inline Result update();
    inline Result updateInterrupted();
    inline Result updatePolled();
    inline Result singleShotMeasure      (Ens220_Sensor sensor = ENS220_SENSOR_TEMPERATURE_AND_PRESSURE, ScioSense_Ens220* other = nullptr); // invokes a single shot mode measuring for the given sensor
    inline Result startContinuousMeasure (Ens220_Sensor sensor = ENS220_SENSOR_TEMPERATURE_AND_PRESSURE, ScioSense_Ens220* other = nullptr); // starts the continuous measure mode for the given sensor
    inline Result stopContinuousMeasure();                                                          // stops the continuous measure mode. for now, this calles reset
    inline Result setHighPower(bool enable);                                                        // immediately en-/disables high power mode. (this is not a setting but a direct write call)
    inline Result reset();

public:
    void enableDebugging(Stream& debugStream);                                                      // Enables the debug log. The output is written to the given debugStream
    void disableDebugging();                                                                        // Stops the debug log if enabled. Does nothing otherwise.

public:
    inline void             waitInterrupt();
    inline void             waitStandbyTime();
    inline void             waitSingleShot();
    inline uint16_t         getTempRaw() const;                                                     // Returns raw temperature data
    inline float            getTempKelvin();                                                        // Converts and returns temperature data in Kelvin
    inline float            getTempCelsius();                                                       // Converts and returns temperature data in Celsius
    inline float            getTempFahrenheit();                                                    // Converts and returns temperature data in Fahrenheit
    inline uint32_t         getPressureRaw();                                                       // Returns raw pressure data
    inline uint32_t         getPressureRaw(const uint8_t& fifoIndex = 255) const;                   // Returns raw FIFO pressure data; returns result of getPressureRaw() if fifoIndex(0-31) is > 31
    inline float            getPressurePascal();                                                    // Converts and returns pressure data in pascal
    inline float            getPressurePascal(uint8_t fifoIndex);                                   // Converts and returns FIFO pressure data in pascal; returns result of getPressurePascal() if fifoIndex(0-31) is > 31
    inline float            getPressureHectoPascal();                                               // Converts and returns pressure data in hectopascal
    inline float            getPressureHectoPascal(uint8_t fifoIndex);                              // Converts and returns FIFO pressure data in hectopascal; returns result of getPressureHectoPascal() if fifoIndex(0-31) is > 31
    inline const uint8_t*   getPressureBuffer();                                                    // Returns FIFO buffer pointer (uint8_t[32*3])

public:
    inline uint16_t CalculateMeasurementTimeSingleShot(const Ens220_Sensor& sensor);                                // Calculates the needed conversion time in ms for the first single shot measurement; based on current device settings
    inline float    CalculateDryAirDensity(const uint8_t& fifoIndex = 255) const;                                   // Calculates the dry air density value, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
    inline float    CalculateDryAirDensity(const ScioSense_Ens220& other, const uint8_t& fifoIndex = 255) const;    // Calculates the average dry air density value, between both devices, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
    inline float    CalculateHeightDifference(const ScioSense_Ens220& other, const uint8_t& fifoIndex = 255) const; // Calculates the height difference between both devices, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
    inline float    CalculateHeightDifference(const float& dryAirDensity, const ScioSense_Ens220& other, const uint8_t& fifoIndex = 255) const; // Calculates the height difference between both devices, based on the current device state using the passed dry air density. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
    inline float    CalculateHeightDifference(const float& dryAirDensity, const uint32_t& p1Raw, const uint32_t& p2Raw) const;  // Calculates the height difference

public:
    inline Ens220_DataStatus        getDataStatus();                                                // Returns current DataStatus (DATA_STAT); only valid if InterfaceConfiguration::InterruptEnable is not set
    inline bool                     hasDataStatusFlag(Ens220_DataStatus flag);                      // Returns true if flag is set in DATA_STAT
    inline Ens220_InterruptStatus   getInterruptStatus();                                           // Returns current InterruptStatus (INT_STAT); only valid if InterfaceConfiguration::InterruptEnable is set
    inline bool                     hasInterruptStatusFlag(Ens220_InterruptStatus flag);            // Returns true if flag is set in DATA_STAT

public:
    inline Result                           writeConfiguration();                                   // Writes settings to the device. must be caled before measurement
    inline const uint8_t*                   getRawConfiguration();                                  // Returns raw configuration registers data (uint8_t[14]). Can be used to save a configuration or configure a second device
    inline uint8_t                          getFifoThreshold();                                     // Returns configured FIFO threshold
    inline Ens220_PressureConversionTime    getPressureConversionTime();                            // Returns configured PressureConversionTime (P_CONV)
    inline Ens220_Oversampling              getOversamplingOfPressure();                            // Returns configured pressure oversampling value (OVSP)
    inline Ens220_Oversampling              getOversamplingOfTemperature ();                        // Returns configured temperature oversampling value (OVST)
    inline uint32_t                         getUID();                                               // returns UID as uint32_t

public:
    inline void setDefaultConfiguration();                                                          // Reset internal configuration to default values
    inline void setRawConfiguration(const uint8_t* config);                                         // Load other configuration uint8_t[14]
    inline void setPressureDataPath(const Ens220_PressureDataPath& dataPath);                       // Set FIFO_MODE (MODE_CFG register)
    inline void setPressureConversionTime(const Ens220_PressureConversionTime& time);               // Set P_CONV (MEAS_CFG register)
    inline void setPressureTemperatureRatio(const Ens220_PressureTemperatureRatio& ratio);          // Set PT_RATE (MEAS_CFG register)
    inline void setStandbyTime(const Ens220_StandbyTime& time);                                     // Set STBY_T (STBY_CFG register)
    inline void setOversamplingOfPressure(const Ens220_Oversampling& n);                            // Set OVSP (OVS_CFG register)
    inline void setOversamplingOfTemperature (const Ens220_Oversampling& n);                        // Set OVST (OVS_CFG register)
    inline void setMovingAverageSamples(const Ens220_MovingAverageSamples& n);                      // Set MAVG (MAVG_CFG register)
    inline void setInterfaceConfiguration(const Ens220_InterfaceConfiguration& interfaceConf);      // Set INTF_CFG register
    inline void setInterruptConfiguration(const Ens220_InterruptConfiguration& inerruptConf);       // Set INT_CFG register
    inline void setLowPressThreshold(const uint32_t& threshold);                                    // Set low-pressure threshold (PRESS_LO registers)
    inline void setHighPressThreshold(const uint32_t& threshold);                                   // Set high-pressure threshold (PRESS_HI registers)
    inline void setFifoThreshold(const uint8_t& threshold);                                         // Set FP_FILL_TH ( FIFO_CFG register)
    inline void setInterruptPin(int p);                                                             // Set INTn interrupt pin

public:
    inline Result read(const Ens220_RegisterAddress address, uint8_t* data, size_t size);
    inline Result write(const Ens220_RegisterAddress address, uint8_t* data, size_t size);

private:
    const char* debugPrefix= "ENS220 debug -- ";
    inline const char* toString(const Result& result);
    inline void debug(const char* msg);
    inline void debug(const char* msg, Result& result);
    inline void debug(const char* msg, uint8_t* data, size_t size, Result& result);
    template<class T> inline void debug(const char* msg, T data);

protected:
    ScioSense_Arduino_I2c_Config        i2cConfig;
    ScioSense_Arduino_Ens220_Spi_Config spiConfig;
    uint8_t interruptPin;

    private:
    Stream* debugStream;
};

#include "ens220.inl.h"

#endif //SCIOSENSE_ENS220_H