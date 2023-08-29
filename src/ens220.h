#ifndef SCIOSENSE_ENS220_H
#define SCIOSENSE_ENS220_H

#include <stdint.h>

#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>

#include "utils.h"

namespace ScioSense
{
    class ENS220
    {
    public:

        // Addresses of the ENS220 registers (named to match the datasheet)
        enum class RegisterAddress : uint8_t
        {
            PART_ID             = 0x00,             // 16 bit Part ID: 0x2X 0x03
            UID                 = 0x02,             // 32 bit unique device identifier
            MODE_CFG            = 0x06,             // Device configuration
            MEAS_CFG            = 0x07,             // Conversion time, P/T rate
            STBY_CFG            = 0x08,             // Standby time configuration
            OVS_CFG             = 0x09,             // Oversampling settings
            MAVG_CFG            = 0x0A,             // Moving average config.
            INTF_CFG            = 0x0B,             // Interface configuration (SPI3)
            INT_CFG             = 0x0C,             // Interrupt mask configuration
            PRESS_LO_XL         = 0x0D,             // Low press.  threshold [7:0]
            PRESS_LO_L          = 0x0E,             // Low press.  threshold [15:8]
            PRESS_LO_H          = 0x0F,             // Low press.  threshold [23:16]
            PRESS_HI_XL         = 0x10,             // High press. threshold [7:0]
            PRESS_HI_L          = 0x11,             // High press. threshold [15:8]
            PRESS_HI_H          = 0x12,             // High press. threshold [23:16]
            FIFO_CFG            = 0x13,             // FIFO configuration
            DATA_STAT           = 0x14,             // Measurement data status
            FIFO_STAT           = 0x15,             // FIFO status
            INT_STAT            = 0x16,             // Interrupt status
            PRESS_OUT_XL        = 0x17,             // Pressure value [7:0]
            PRESS_OUT_L         = 0x18,             // Pressure value [15:8]
            PRESS_OUT_H         = 0x19,             // Pressure value [23:16]
            TEMP_OUT_L          = 0x1A,             // Temperature value [7:0]
            TEMP_OUT_H          = 0x1B,             // Temperature value [15:8]
            PRESS_OUT_F_XL      = 0x27,             // FIFO Pressure value [7:0]
            PRESS_OUT_F_L       = 0x28,             // FIFO Pressure value [15:8]
            PRESS_OUT_F_H       = 0x29,             // FIFO Pressure value [23:16]
        };

        // Device configuration Flags used internally. Part of MODE_CFG register.
        enum class ModeConfiguration : uint8_t
        {
            HighPower           = 1 << 7,           // High Power enable
            Start               = 1 << 4,           // Operating mode configuration
            Reset               = 1 << 3,           // Device reset to the power-on configuration. automatically cleared
            MeasureTemperature  = 1 << 1,           // Enable temperature measurements
            MeasurePressure     = 1 << 0,           // Enable pressure measurements

            Mask                = ( HighPower | Start | Reset | MeasureTemperature | MeasurePressure )
        };

        // Device configuration Flags. Used to start a measurement with the selected sensor. Subset of ModeConfiguration
        enum class Sensor: uint8_t
        {
            Temperature                 = (uint8_t)ModeConfiguration::MeasureTemperature,
            Pressure                    = (uint8_t)ModeConfiguration::MeasurePressure,
            TemperatureAndPressure      = Temperature | Pressure
        };

        // Pressure data path configuration. Part of MODE_CFG register (FIFO_MODE).
        enum class PressureDataPath : uint8_t
        {
            Direct          = 0b00,                 // Direct path. FIFO disabled
            Fifo            = 1 << 5,               // FIFO enabled
            MovingAverage   = 1 << 6,               // FIFO used to calculate moving average

            Mask            = (Fifo | MovingAverage)
        };

        // Pressure ADC conversion time. part of MEAS_CFG register (P_CONV);
        enum class PressureConversionTime : uint8_t
        {
            T_4_1   = 0,                            // First conversion 4ms;  Next conversions 1ms
            T_8_2   = 1,                            // First conversion 8ms;  Next conversions 2ms
            T_16_4  = 2                             // First conversion 16ms; Next conversions 4ms
        };

        // The ratio between P and T measurements as produced by the measurement engine. Part of MEAS_CFG register (PT_RATE).
        enum class PressureTemperatureRatio : uint8_t
        {
            PT_1    = 0,                            // 1P   / 1T
            PT_4    = 1,                            // 4P   / 1T
            PT_8    = 2,                            // 8P   / 1T
            PT_16   = 3,                            // 16P  / 1T
            PT_32   = 4,                            // 32P  / 1T
            PT_64   = 5,                            // 64P  / 1T
            PT_128  = 6,                            // 128P / 1T
            PT_256  = 7,                            // 256P / 1T
        };

        // Standby configuration (STBY_CFG)
        enum class StandbyTime : uint8_t
        {
            ContinousOperation  = 0,                // Continuous operation
            OneShotOperation    = 1,                // One-shot operation (device returns to idle after one measurement is produced by the measurement engine)
            T_10                = 2,                //  10ms
            T_20                = 3,                //  20ms
            T_30                = 4,                //  30ms
            T_50                = 5,                //  50ms
            T_100               = 6,                // 100ms
            T_250               = 7,                // 250ms
            T_500               = 8,                // 500ms
            T_750               = 9,                // 750ms
            T_1000              = 10,               //    1s
            T_2000              = 11,               //    2s
            T_5000              = 12,               //    5s
            T_10000             = 13,               //   10s
            T_60000             = 14,               //   60s
            T_600000            = 15                //  600s
        };

        //Oversampling. Number of averages. Part of OVS_CFG register.
        enum class Oversampling : uint8_t
        {
            N_1     = 0,                            //   1
            N_2     = 1,                            //   2
            N_4     = 2,                            //   4
            N_8     = 3,                            //   8
            N_16    = 4,                            //  16
            N_32    = 5,                            //  32
            N_64    = 6,                            //  64
            N_128   = 7,                            // 128
        };

        //Moving average configuration. Part of MAVG_CFG register. Controls the number of samples used by the moving average filter (MAVG).
        enum class MovingAverageSamples : uint8_t
        {
            N_1     = 0,                            //  1
            N_2     = 1,                            //  2
            N_4     = 2,                            //  4
            N_8     = 3,                            //  8
            N_16    = 4,                            // 16
            N_32    = 5,                            // 32
        };

        // Host interface configuration register (INTF_CFG).
        enum class InterfaceConfiguration : uint8_t
        {
            InterruptEnable     = 1 << 2,           // Interrupt enable
            InterruptPolarity   = 1 << 1,           // Interrupt polarity; 0b0: HIGH; 0b1: LOW
            Spi3WireMode        = 1 << 0            // SPI mode control 0b1: SPI works in 3-wire mode
        };

        // Interrupt configuration register (INT_CFG).
        enum class InterruptConfiguration : uint8_t
        {
            TemperatureDataReady    = 1 << 6,       // Interrupt is triggered when new temperature data becomes available
            PressureFifoLevelHigh   = 1 << 5,       // Interrupt is triggered when the filling level of the pressure FIFO becomes greater than the programmed threshold (FP_FILL_TH)
            PressureFifoFull        = 1 << 4,       // Interrupt is triggered when the pressure FIFO becomes full (32 values)
            PressureFifoEmpty       = 1 << 3,       // Interrupt is triggered when the pressure FIFO becomes empty
            PressureDataReady       = 1 << 2,       // interrupt is triggered when new pressure data becomes available
            PressureHigh            = 1 << 1,       // Interrupt is triggered when the pressure is greater than the programmed pressure high threshold (PRESS_HI)
            PressureLow             = 1 << 0,       // Interrupt is triggered when the pressure is lower than the programmed pressure low threshold (PRESS_LO)
        };

        // FIFO configuration register flags. Part of FIFO_CFG.
        enum class FifoConfiguration : uint8_t
        {
            Clear = 1 << 5                          // the content of the FIFO is cleared; FP_CLEAR is automatically cleared.
        };

        // Data status register (DATA_STAT).
        enum class DataStatus : uint8_t
        {
            PressureOverwrite       = 1 << 3,       // This bit is set whenever a new pressure measurement is produced by the measurement engine while the pressure FIFO was already full.
            TemperatureOverwrite    = 1 << 2,       // This bit is set when a new temperature measurement is produced by the measurement engine and the previous data was not read.
            PressureReady           = 1 << 1,       // This bit is set when new pressure data becomes available.
            TemperatureReady        = 1 << 0,       // This bit is set when new temperature data becomes available.
            None                    = 0
        };

        // FIFO status register (FIFO_STAT).
        enum FifoStatus : uint8_t
        {
            Full                    = 1 << 2,       // is set when the FIFO is enabled and is full (32 elements).
            Empty                   = 1 << 1,       // is set when the FIFO is enabled and is empty (0 elements).
            ReachedThreshold        = 1 << 0,       // is set when is enabled and the number of elements is greater than threshold (FP_FILL_TH).
        };

        // Interrupt status register (INT_STAT).
        enum InterruptStatus : uint8_t
        {
            General                 = 1 << 7,       // (IA) is set if any of the interrupt flags relative to sources enabled in INT_CFG are set;
            Temperature             = 1 << 6,       // (TR) is set when a new temperature measurement is produced by the measurement engine
            FifoReachedThreshold    = 1 << 5,       // (FH) is set when FP_FILL > FP_FILL_TH.
            FifoFull                = 1 << 4,       // (FF) is set when the FIFO becomes full.
            FifoEmpty               = 1 << 3,       // (FE) is set when FIFO becomes empty as effect of a dataread.
            Pressure                = 1 << 2,       // (PR) is set when new pressure measurement is produced.
            HighPressure            = 1 << 1,       // (PH) is set when the measurement engine produces a pressure measurement that is greater than PRESS_HI.
            LowPressure             = 1 << 0,       // (PL) is set when the measurement engine produces a pressure measurement that is lower than PRESS _LO

            None                    = 0
        };

        enum class Result : uint8_t
        {
            NotAllowed              = 5,            // (unused) The requested command is not allowed.
            IOError                 = 4,            // There was an IO communication error, read/write the stream failed.
            ChecksumError           = 3,            // (unused) The value was read, but the checksum over the payload (valid and data) does not match.
            Invalid                 = 2,            // The value was read, but the data is invalid.
            Ok                      = 1             // All OK; The value was read, the checksum matches, and data is valid.
        };

        // BufferInfo defines internally used buffer and data indices and sizes (in bytes)
        struct BufferInfo
        {
            static constexpr uint8_t ConfigurationSize  = 14;
            static constexpr uint8_t PressureSize       = 3;
            static constexpr uint8_t TemperatureSize    = 2;
            static constexpr uint8_t FifoSize           = PressureSize * 32;
            static constexpr uint8_t UpdateBufferSize   = TemperatureSize + PressureSize + FifoSize;

            static constexpr uint8_t PressureIndex      = 0;
            static constexpr uint8_t TemperatureIndex   = PressureIndex + PressureSize;
            static constexpr uint8_t FifoIndex          = TemperatureIndex + TemperatureSize;

            static constexpr uint8_t MODE_CFG           = (uint8_t)RegisterAddress::MODE_CFG    - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t MEAS_CFG           = (uint8_t)RegisterAddress::MEAS_CFG    - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t STBY_CFG           = (uint8_t)RegisterAddress::STBY_CFG    - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t OVS_CFG            = (uint8_t)RegisterAddress::OVS_CFG     - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t MAVG_CFG           = (uint8_t)RegisterAddress::MAVG_CFG    - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t INTF_CFG           = (uint8_t)RegisterAddress::INTF_CFG    - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t INT_CFG            = (uint8_t)RegisterAddress::INT_CFG     - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t PRESS_LO_XL        = (uint8_t)RegisterAddress::PRESS_LO_XL - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t PRESS_LO_L         = (uint8_t)RegisterAddress::PRESS_LO_L  - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t PRESS_LO_H         = (uint8_t)RegisterAddress::PRESS_LO_H  - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t PRESS_HI_XL        = (uint8_t)RegisterAddress::PRESS_HI_XL - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t PRESS_HI_L         = (uint8_t)RegisterAddress::PRESS_HI_L  - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t PRESS_HI_H         = (uint8_t)RegisterAddress::PRESS_HI_H  - (uint8_t)RegisterAddress::MODE_CFG;
            static constexpr uint8_t FIFO_CFG           = (uint8_t)RegisterAddress::FIFO_CFG    - (uint8_t)RegisterAddress::MODE_CFG;
        };

    public:
        ENS220();
        ~ENS220();

    public:
        bool begin(Utils::IoInterface<RegisterAddress, Result>* ioInterface, bool spi3WireMode= false);                 // Connnects to ENS220 using the given IoInterface object and reads PART_ID, and UID; returns the result of isConnected()
        bool isConnected();                                                                                             // checks if the read PART_ID matches the expected value; returns true, if so.

    public:
        inline Result update();
        Result updateInterrupted();
        Result updatePolled();
        Result singleShotMeasure        (Sensor sensor = Sensor::TemperatureAndPressure, ENS220* other = nullptr);      // invokes a single shot mode measuring for the given sensor
        Result startContinuousMeasure   (Sensor sensor = Sensor::TemperatureAndPressure, ENS220* other = nullptr);      // starts the continuous measure mode for the given sensor
        Result stopContinuousMeasure();                                                                                 // stops the continuous measure mode. for now, this calles reset
        Result setHighPower(bool enable);                                                                               // immediately en-/disables high power mode. (this is not a setting but a direct write call)
        Result reset();

    public:
        void enableDebugging(Stream& debugStream);                                                                      // Enables the debug log. The output is written to the given debugStream
        void disableDebugging();                                                                                        // Stops the debug log if enabled. Does nothing otherwise.

    public:
        inline void     waitInterrupt();
        inline void     waitStandbyTime();
        inline void     waitSingleShot();
        inline uint16_t getTempRaw() const;                                                                             // Returns raw temperature data
        inline float    getTempKelvin();                                                                                // Converts and returns temperature data in Kelvin
        inline float    getTempCelsius();                                                                               // Converts and returns temperature data in Celsius
        inline float    getTempFahrenheit();                                                                            // Converts and returns temperature data in Fahrenheit
        inline uint32_t getPressureRaw();                                                                               // Returns raw pressure data
        inline uint32_t getPressureRaw(const uint8_t& fifoIndex = 255) const;                                           // Returns raw FIFO pressure data; returns result of getPressureRaw() if fifoIndex(0-31) is > 31
        inline float    getPressurePascal();                                                                            // Converts and returns pressure data in pascal
        inline float    getPressurePascal(uint8_t fifoIndex);                                                           // Converts and returns FIFO pressure data in pascal; returns result of getPressurePascal() if fifoIndex(0-31) is > 31
        inline float    getPressureHectoPascal();                                                                       // Converts and returns pressure data in hectopascal
        inline float    getPressureHectoPascal(uint8_t fifoIndex);                                                      // Converts and returns FIFO pressure data in hectopascal; returns result of getPressureHectoPascal() if fifoIndex(0-31) is > 31
        inline uint8_t* getPressureBuffer();                                                                            // Returns FIFO buffer pointer (uint8_t[32*3])

    public:
        inline uint16_t CalculateMeasurementTimeSingleShot(const Sensor& sensor);                                                           // Calculates the needed conversion time in ms for the first single shot measurement; based on current device settings
        inline float    CalculateDryAirDensity(const uint8_t& fifoIndex = 255) const;                                                       // Calculates the dry air density value, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
        inline float    CalculateDryAirDensity(const ENS220& other, const uint8_t& fifoIndex=255) const;                                    // Calculates the average dry air density value, between both devices, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
        inline float    CalculateHeightDifference(const ENS220& other, const uint8_t& fifoIndex = 255) const;                               // Calculates the height difference between both devices, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
        inline float    CalculateHeightDifference(const float& dryAirDensity, const ENS220& other, const uint8_t& fifoIndex = 255) const;   // Calculates the height difference between both devices, based on the current device state using the passed dry air density. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
        inline float    CalculateHeightDifference(const float& dryAirDensity, const uint32_t& p1Raw, const uint32_t& p2Raw) const;          // Calculates the height difference

    public:
        inline DataStatus       getDataStatus();                                                                        // Returns current DataStatus (DATA_STAT); only valid if InterfaceConfiguration::InterruptEnable is not set
        inline InterruptStatus   getInterruptStatus();                                                                    // Returns current InterruptStatus (INT_STAT); only valid if InterfaceConfiguration::InterruptEnable is set

    //Device Configuration
    private:
        uint8_t configurationData[BufferInfo::ConfigurationSize];

    public:
        void                            writeConfiguration();                                                           // Writes settings to the device. must be caled before measurement
        inline const uint8_t*           getRawConfiguration();                                                          // Returns raw configuration registers data (uint8_t[14]). Can be used to save a configuration or configure a second device
        inline uint8_t                  getFifoThreshold();                                                             // Returns configured FIFO threshold
        inline PressureConversionTime   getPressureConversionTime();                                                    // Returns configured PressureConversionTime (P_CONV)
        inline Oversampling             getOversamplingOfPressure();                                                    // Returns configured pressure oversampling value (OVSP)
        inline Oversampling             getOversamplingOfTemperature ();                                                // Returns configured temperature oversampling value (OVST)
        inline uint32_t                 getUID();                                                                       // returns UID as uint32_t

    public:
        inline void setDefaultConfiguration();                                                                          // Reset internal configuration to default values
        inline void setRawConfiguration(const uint8_t* config);                                                         // Load other configuration uint8_t[14]
        inline void setPressureDataPath(const PressureDataPath& dataPath);                                              // Set FIFO_MODE (MODE_CFG register)
        inline void setPressureConversionTime(const PressureConversionTime& time);                                      // Set P_CONV (MEAS_CFG register)
        inline void setPressureTemperatureRatio(const PressureTemperatureRatio& ratio);                                 // Set PT_RATE (MEAS_CFG register)
        inline void setStandbyTime(const StandbyTime& time);                                                            // Set STBY_T (STBY_CFG register)
        inline void setOversamplingOfPressure(const Oversampling& n);                                                   // Set OVSP (OVS_CFG register)
        inline void setOversamplingOfTemperature (const Oversampling& n);                                               // Set OVST (OVS_CFG register)
        inline void setMovingAverageSamples(const MovingAverageSamples& n);                                             // Set MAVG (MAVG_CFG register)
        inline void setInterfaceConfiguration(const InterfaceConfiguration& interfaceConf);                             // Set INTF_CFG register
        inline void setInterruptConfiguration(const InterruptConfiguration& inerruptConf);                              // Set INT_CFG register
        inline void setLowPressThreshold(const uint32_t& threshold);                                                    // Set low-pressure threshold (PRESS_LO registers)
        inline void setHighPressThreshold(const uint32_t& threshold);                                                   // Set high-pressure threshold (PRESS_HI registers)
        inline void setFifoThreshold(const uint8_t& threshold);                                                         // Set FP_FILL_TH ( FIFO_CFG register)
        inline void setInterruptPin(int p);                                                                             // Set INTn interrupt pin

    private:
        Result setupSingleShotMeasure(Sensor sensor);

    private:
        inline Result read(RegisterAddress address, uint8_t* data, size_t size);
        inline Result write(RegisterAddress address, uint8_t* data, size_t size);
        template<class T> Result read(RegisterAddress address, T& data);
        template<class T> Result write(RegisterAddress address, T data);

    private:
        const char* debugPrefix= "ENS220 debug -- ";
        inline const char* toString(const Result& result);
        inline void debug(const char* msg);
        inline void debug(const char* msg, Result& result);
        inline void debug(const char* msg, uint8_t* data, size_t size, Result& result);
        template<class T> inline void debug(const char* msg, T data);

    private:
        uint8_t buffer[BufferInfo::UpdateBufferSize];
        DataStatus dataStatus;
        StandbyTime standbyTime;
        InterruptStatus interruptStatus;
        uint8_t interruptPin;
        ModeConfiguration modeConfiguration;
        uint16_t partId;
        uint32_t uid;

    private:
        Utils::IoInterface<RegisterAddress, Result>* ioInterface;
        bool spi3WireMode;
        Stream* debugStream;
    };

    DEFINE_ENUM_FLAG_OPERATORS(ENS220::ModeConfiguration, uint8_t);
    DEFINE_ENUM_FLAG_OPERATORS(ENS220::InterruptConfiguration, uint8_t);
    DEFINE_ENUM_FLAG_OPERATORS(ENS220::PressureDataPath, uint8_t);
    DEFINE_ENUM_FLAG_OPERATORS(ENS220::DataStatus, uint8_t);
    DEFINE_ENUM_FLAG_OPERATORS(ENS220::InterfaceConfiguration, uint8_t);
    DEFINE_ENUM_FLAG_OPERATORS(ENS220::InterruptStatus, uint8_t);
}

#include "ens220.inl"

#endif //SCIOSENSE_ENS220_H