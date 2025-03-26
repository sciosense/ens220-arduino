#ifndef SCIOSENSE_ENS220_C_H
#define SCIOSENSE_ENS220_C_H

#include "ScioSense_Ens220_defines.h"

#include <stdbool.h>
#include <inttypes.h>

typedef struct ScioSense_Ens220_IO
{
    Result  (*read)     (void* config, const uint16_t address, uint8_t* data, const size_t size);
    Result  (*write)    (void* config, const uint16_t address, uint8_t* data, const size_t size);
    void    (*wait)     (const uint32_t ms);
    bool spi3WireMode;
    void* config;
} ScioSense_Ens220_IO;

typedef struct ScioSense_Ens220
{
    ScioSense_Ens220_IO         io;
    uint8_t                     dataBuffer          [ENS220_BUFFER_INFO_UPDATE_BUFFER_SIZE];
    uint8_t                     configurationBuffer [ENS220_BUFFER_INFO_CONFIGURATION_SIZE];
    uint16_t                    partId;
    uint32_t                    uid;
    Ens220_DataStatus           dataStatus;
    Ens220_StandbyTime          standbyTime;
    Ens220_InterruptStatus      interruptStatus;
    Ens220_ModeConfiguration    modeConfiguration;
} ScioSense_Ens220;

static inline Result Ens220_Init                          (ScioSense_Ens220* ens220);                 // Resets the device to IDLE and reads PartID and FirmwareVersion
static inline Result Ens220_Reset                         (ScioSense_Ens220* ens220);                 // Device reset to the power-on configuration.
static inline Result Ens220_Update                        (ScioSense_Ens220* ens220);                 // Calls either UpdatePolled or UpdateInterrupted depending on the configuration.
static inline Result Ens220_UpdateInterrupted             (ScioSense_Ens220* ens220);                 // Reads INT_STAT, Reads Data if flags are set.
static inline Result Ens220_UpdatePolled                  (ScioSense_Ens220* ens220);                 // Reads DATA_STAT, Reads Data if flags are set.
static inline Result Ens220_ReadPartId                    (ScioSense_Ens220* ens220);                 // Reads the PART_ID of the device
static inline Result Ens220_ReadUid                       (ScioSense_Ens220* ens220);                 // Reads the UID of the device
static inline Result Ens220_SetHighPower                  (ScioSense_Ens220* ens220, bool enable);    // immediately en-/disables high power mode. (this is not a setting but a direct write call)
static inline Result Ens220_SingleShotMeasure             (ScioSense_Ens220* ens220, Ens220_Sensor sensor, ScioSense_Ens220* other);   // invokes a single shot mode measuring for the given sensor
static inline Result Ens220_StartContinuousMeasure        (ScioSense_Ens220* ens220, Ens220_Sensor sensor, ScioSense_Ens220* other);   // starts the continuous measure mode for the given sensor
static inline Result Ens220_StopContinuousMeasure         (ScioSense_Ens220* ens220);                 // stops the continuous measure mode. for now, this calles reset

static inline void   Ens220_SetDefaultConfiguration       (ScioSense_Ens220* ens220);                                                        // Reset internal configuration to default values
static inline void   Ens220_SetRawConfiguration           (ScioSense_Ens220* ens220, const uint8_t* config);                                 // Load other configuration uint8_t[14]
static inline void   Ens220_SetPressureDataPath           (ScioSense_Ens220* ens220, const Ens220_PressureDataPath dataPath);                // Set FIFO_MODE (MODE_CFG register)
static inline void   Ens220_SetPressureConversionTime     (ScioSense_Ens220* ens220, const Ens220_PressureConversionTime time);              // Set P_CONV (MEAS_CFG register)
static inline void   Ens220_SetPressureTemperatureRatio   (ScioSense_Ens220* ens220, const Ens220_PressureTemperatureRatio ratio);           // Set PT_RATE (MEAS_CFG register)
static inline void   Ens220_SetStandbyTime                (ScioSense_Ens220* ens220, const Ens220_StandbyTime time);                         // Set STBY_T (STBY_CFG register)
static inline void   Ens220_SetOversamplingOfPressure     (ScioSense_Ens220* ens220, const Ens220_Oversampling n);                           // Set OVSP (OVS_CFG register)
static inline void   Ens220_SetOversamplingOfTemperature  (ScioSense_Ens220* ens220, const Ens220_Oversampling n);                           // Set OVST (OVS_CFG register)
static inline void   Ens220_SetMovingAverageSamples       (ScioSense_Ens220* ens220, const Ens220_MovingAverageSamples n);                   // Set MAVG (MAVG_CFG register)
static inline void   Ens220_SetInterfaceConfiguration     (ScioSense_Ens220* ens220, const Ens220_InterfaceConfiguration interfaceConf);     // Set INTF_CFG register
static inline void   Ens220_SetInterruptConfiguration     (ScioSense_Ens220* ens220, const Ens220_InterruptConfiguration inerruptConf);      // Set INT_CFG register
static inline void   Ens220_SetLowPressThreshold          (ScioSense_Ens220* ens220, const uint32_t threshold);                              // Set low-pressure threshold (PRESS_LO registers)
static inline void   Ens220_SetHighPressThreshold         (ScioSense_Ens220* ens220, const uint32_t threshold);                              // Set high-pressure threshold (PRESS_HI registers)
static inline void   Ens220_SetFifoThreshold              (ScioSense_Ens220* ens220, const uint8_t threshold);                               // Set FP_FILL_TH ( FIFO_CFG register)
static inline Result Ens220_WriteConfiguration            (ScioSense_Ens220* ens220);                                                        // Writes settings to the device. must be caled before measurement

static inline bool                            Ens220_IsConnected                  (const ScioSense_Ens220* ens220);                           // checks if the read PART_ID matches the expected value; returns true, if so.
static inline uint16_t                        Ens220_GetTempRaw                   (const ScioSense_Ens220* ens220);                           // Returns raw temperature data
static inline float                           Ens220_GetTempKelvin                (const ScioSense_Ens220* ens220);                           // Converts and returns temperature data in Kelvin
static inline float                           Ens220_GetTempCelsius               (const ScioSense_Ens220* ens220);                           // Converts and returns temperature data in Celsius
static inline float                           Ens220_GetTempFahrenheit            (const ScioSense_Ens220* ens220);                           // Converts and returns temperature data in Fahrenheit
static inline uint32_t                        Ens220_GetPressureRaw               (const ScioSense_Ens220* ens220);                           // Returns raw pressure data
static inline uint32_t                        Ens220_GetPressureRawFifo           (const ScioSense_Ens220* ens220, const uint8_t fifoIndex);  // Returns raw FIFO pressure data; returns result of getPressureRaw() if fifoIndex(0-31) is > 31
static inline float                           Ens220_GetPressurePascal            (const ScioSense_Ens220* ens220);                           // Converts and returns pressure data in pascal
static inline float                           Ens220_GetPressurePascalFifo        (const ScioSense_Ens220* ens220, uint8_t fifoIndex);        // Converts and returns FIFO pressure data in pascal; returns result of getPressurePascal() if fifoIndex(0-31) is > 31
static inline float                           Ens220_GetPressureHectoPascal       (const ScioSense_Ens220* ens220);                           // Converts and returns pressure data in hectopascal
static inline float                           Ens220_GetPressureHectoPascalFifo   (const ScioSense_Ens220* ens220, uint8_t fifoIndex);        // Converts and returns FIFO pressure data in hectopascal; returns result of getPressureHectoPascal() if fifoIndex(0-31) is > 31
static inline const uint8_t*                  Ens220_GetPressureBuffer            (const ScioSense_Ens220* ens220);                           // Returns FIFO buffer pointer (uint8_t[32*3])
static inline Ens220_DataStatus               Ens220_GetDataStatus                (const ScioSense_Ens220* ens220);                           // Returns current DataStatus (DATA_STAT); only valid if InterfaceConfiguration::InterruptEnable is not set
static inline Ens220_InterruptStatus          Ens220_GetInterruptStatus           (const ScioSense_Ens220* ens220);                           // Returns current InterruptStatus (INT_STAT); only valid if InterfaceConfiguration::InterruptEnable is set
static inline const uint8_t*                  Ens220_GetRawConfiguration          (const ScioSense_Ens220* ens220);                           // Returns raw configuration registers data (uint8_t[14]). Can be used to save a configuration or configure a second device
static inline uint8_t                         Ens220_GetFifoThreshold             (const ScioSense_Ens220* ens220);                           // Returns configured FIFO threshold
static inline Ens220_PressureConversionTime   Ens220_GetPressureConversionTime    (const ScioSense_Ens220* ens220);                           // Returns configured PressureConversionTime (P_CONV)
static inline Ens220_Oversampling             Ens220_GetOversamplingOfPressure    (const ScioSense_Ens220* ens220);                           // Returns configured pressure oversampling value (OVSP)
static inline Ens220_Oversampling             Ens220_GetOversamplingOfTemperature (const ScioSense_Ens220* ens220);                           // Returns configured temperature oversampling value (OVST)
static inline uint32_t                        Ens220_GetUid                       (const ScioSense_Ens220* ens220);                           // returns UID as uint32_t

static inline void        Ens220_WaitStandbyTime                      (const ScioSense_Ens220* ens220);
static inline void        Ens220_WaitSingleShot                       (const ScioSense_Ens220* ens220);
static inline uint16_t    Ens220_CalculateMeasurementTimeSingleShot   (const ScioSense_Ens220* ens220, const Ens220_Sensor sensor);            // Calculates the needed conversion time in ms for the first single shot measurement; based on current device settings
static inline float       Ens220_CalculateDryAirDensity               (const ScioSense_Ens220* ens220, const uint8_t fifoIndex);               // Calculates the dry air density value, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
static inline float       Ens220_CalculateDryAirDensity2              (const ScioSense_Ens220* ens220, const ScioSense_Ens220* other, const uint8_t fifoIndex);    // Calculates the average dry air density value, between both devices, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
static inline float       Ens220_CalculateHeightDifference            (const ScioSense_Ens220* ens220, const ScioSense_Ens220* other, const uint8_t fifoIndex);    // Calculates the height difference between both devices, based on the current device state. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
static inline float       Ens220_CalculateHeightDifference2           (const ScioSense_Ens220* ens220, const float dryAirDensity, const ScioSense_Ens220* other, const uint8_t fifoIndex);    // Calculates the height difference between both devices, based on the current device state using the passed dry air density. if fifoIndex is out of range (>31), PressureDataPath::Direct is assumed
static inline float       Ens220_CalculateHeightDifference3           (const float dryAirDensity, const uint32_t p1Raw, const uint32_t p2Raw); // Calculates the height difference


#include "ScioSense_Ens220.inl.h"
#endif // SCIOSENSE_ENS220_C_H