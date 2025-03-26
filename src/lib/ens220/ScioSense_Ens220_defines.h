#ifndef SCIOSENSE_ENS220_DEFINES_C_H
#define SCIOSENSE_ENS220_DEFINES_C_H

#include <inttypes.h>

// Addresses of the ENS220 registers (named to match the datasheet)
typedef uint8_t Ens220_RegisterAddress;
#define ENS220_REGISTER_ADDRESS_PART_ID             (0x00)          // 16 bit Part ID: 0x2X 0x03
#define ENS220_REGISTER_ADDRESS_UID                 (0x02)          // 32 bit unique device identifier
#define ENS220_REGISTER_ADDRESS_MODE_CFG            (0x06)          // Device configuration
#define ENS220_REGISTER_ADDRESS_MEAS_CFG            (0x07)          // Conversion time, P/T rate
#define ENS220_REGISTER_ADDRESS_STBY_CFG            (0x08)          // Standby time configuration
#define ENS220_REGISTER_ADDRESS_OVS_CFG             (0x09)          // Oversampling settings
#define ENS220_REGISTER_ADDRESS_MAVG_CFG            (0x0A)          // Moving average config.
#define ENS220_REGISTER_ADDRESS_INTF_CFG            (0x0B)          // Interface configuration (SPI3)
#define ENS220_REGISTER_ADDRESS_INT_CFG             (0x0C)          // Interrupt mask configuration
#define ENS220_REGISTER_ADDRESS_PRESS_LO_XL         (0x0D)          // Low press.  threshold [7:0]
#define ENS220_REGISTER_ADDRESS_PRESS_LO_L          (0x0E)          // Low press.  threshold [15:8]
#define ENS220_REGISTER_ADDRESS_PRESS_LO_H          (0x0F)          // Low press.  threshold [23:16]
#define ENS220_REGISTER_ADDRESS_PRESS_HI_XL         (0x10)          // High press. threshold [7:0]
#define ENS220_REGISTER_ADDRESS_PRESS_HI_L          (0x11)          // High press. threshold [15:8]
#define ENS220_REGISTER_ADDRESS_PRESS_HI_H          (0x12)          // High press. threshold [23:16]
#define ENS220_REGISTER_ADDRESS_FIFO_CFG            (0x13)          // FIFO configuration
#define ENS220_REGISTER_ADDRESS_DATA_STAT           (0x14)          // Measurement data status
#define ENS220_REGISTER_ADDRESS_FIFO_STAT           (0x15)          // FIFO status
#define ENS220_REGISTER_ADDRESS_INT_STAT            (0x16)          // Interrupt status
#define ENS220_REGISTER_ADDRESS_PRESS_OUT_XL        (0x17)          // Pressure value [7:0]
#define ENS220_REGISTER_ADDRESS_PRESS_OUT_L         (0x18)          // Pressure value [15:8]
#define ENS220_REGISTER_ADDRESS_PRESS_OUT_H         (0x19)          // Pressure value [23:16]
#define ENS220_REGISTER_ADDRESS_TEMP_OUT_L          (0x1A)          // Temperature value [7:0]
#define ENS220_REGISTER_ADDRESS_TEMP_OUT_H          (0x1B)          // Temperature value [15:8]
#define ENS220_REGISTER_ADDRESS_PRESS_OUT_F_XL      (0x27)          // FIFO Pressure value [7:0]
#define ENS220_REGISTER_ADDRESS_PRESS_OUT_F_L       (0x28)          // FIFO Pressure value [15:8]
#define ENS220_REGISTER_ADDRESS_PRESS_OUT_F_H       (0x29)          // FIFO Pressure value [23:16]

// Device configuration Flags used internally. Part of MODE_CFG register.
typedef uint8_t Ens220_ModeConfiguration;
#define ENS220_MODE_CONFIGURATION_HIGH_POWER            (uint8_t)(1 << 7)           // High Power enable
#define ENS220_MODE_CONFIGURATION_START                 (uint8_t)(1 << 4)           // Operating mode configuration
#define ENS220_MODE_CONFIGURATION_RESET                 (uint8_t)(1 << 3)           // Device reset to the power-on configuration. automatically cleared
#define ENS220_MODE_CONFIGURATION_MEASURE_TEMPERATURE   (uint8_t)(1 << 1)           // Enable temperature measurements
#define ENS220_MODE_CONFIGURATION_MEASURE_PRESSURE      (uint8_t)(1 << 0)           // Enable pressure measurements
#define ENS220_MODE_CONFIGURATION_MASK                  (uint8_t)( ENS220_MODE_CONFIGURATION_HIGH_POWER | ENS220_MODE_CONFIGURATION_START | ENS220_MODE_CONFIGURATION_RESET | ENS220_MODE_CONFIGURATION_MEASURE_TEMPERATURE | ENS220_MODE_CONFIGURATION_MEASURE_PRESSURE )

// Device configuration Flags. Used to start a measurement with the selected sensor. Subset of ModeConfiguration
typedef uint8_t Ens220_Sensor;
#define ENS220_SENSOR_TEMPERATURE                   ENS220_MODE_CONFIGURATION_MEASURE_TEMPERATURE
#define ENS220_SENSOR_PRESSURE                      ENS220_MODE_CONFIGURATION_MEASURE_PRESSURE
#define ENS220_SENSOR_TEMPERATURE_AND_PRESSURE      (uint8_t)(ENS220_SENSOR_TEMPERATURE | ENS220_SENSOR_PRESSURE)

// Pressure data path configuration. Part of MODE_CFG register (FIFO_MODE).
typedef uint8_t Ens220_PressureDataPath;
#define ENS220_PRESSURE_DATA_PATH_DIRECT            (uint8_t)( 0b00 )                   // Direct path. FIFO disabled
#define ENS220_PRESSURE_DATA_PATH_FIFO              (uint8_t)(1 << 5)                   // FIFO enabled
#define ENS220_PRESSURE_DATA_PATH_MOVING_AVERAGE    (uint8_t)(1 << 6)                   // FIFO used to calculate moving average
#define ENS220_PRESSURE_DATA_PATH_MASK              (uint8_t)(ENS220_PRESSURE_DATA_PATH_FIFO | ENS220_PRESSURE_DATA_PATH_MOVING_AVERAGE)

// Pressure ADC conversion time. part of MEAS_CFG register (P_CONV);
typedef uint8_t Ens220_PressureConversionTime;
#define ENS220_PRESSURE_CONVERSION_TIME_T_4_1       (0)                            // First conversion 4ms;  Next conversions 1ms
#define ENS220_PRESSURE_CONVERSION_TIME_T_8_2       (1)                            // First conversion 8ms;  Next conversions 2ms
#define ENS220_PRESSURE_CONVERSION_TIME_T_16_4      (2)                            // First conversion 16ms; Next conversions 4ms

// The ratio between P and T measurements as produced by the measurement engine. Part of MEAS_CFG register (PT_RATE).
typedef uint8_t Ens220_PressureTemperatureRatio;
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_1      (0)                            // 1P   / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_4      (1)                            // 4P   / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_8      (2)                            // 8P   / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_16     (3)                            // 16P  / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_32     (4)                            // 32P  / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_64     (5)                            // 64P  / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_128    (6)                            // 128P / 1T
#define ENS220_PRESSURE_TEMPERATURE_RATIO_PT_256    (7)                            // 256P / 1T

// Standby configuration (STBY_CFG)
typedef uint8_t Ens220_StandbyTime;
#define ENS220_STANDBY_TIME_CONTINOUS_OPERATION (0 )               // Continuous operation
#define ENS220_STANDBY_TIME_ONE_SHOT_OPERATION  (1 )               // One-shot operation (device returns to idle after one measurement is produced by the measurement engine)
#define ENS220_STANDBY_TIME_T_10                (2 )               //  10ms
#define ENS220_STANDBY_TIME_T_20                (3 )               //  20ms
#define ENS220_STANDBY_TIME_T_30                (4 )               //  30ms
#define ENS220_STANDBY_TIME_T_50                (5 )               //  50ms
#define ENS220_STANDBY_TIME_T_100               (6 )               // 100ms
#define ENS220_STANDBY_TIME_T_250               (7 )               // 250ms
#define ENS220_STANDBY_TIME_T_500               (8 )               // 500ms
#define ENS220_STANDBY_TIME_T_750               (9 )               // 750ms
#define ENS220_STANDBY_TIME_T_1000              (10)               //    1s
#define ENS220_STANDBY_TIME_T_2000              (11)               //    2s
#define ENS220_STANDBY_TIME_T_5000              (12)               //    5s
#define ENS220_STANDBY_TIME_T_10000             (13)               //   10s
#define ENS220_STANDBY_TIME_T_60000             (14)               //   60s
#define ENS220_STANDBY_TIME_T_600000            (15)               //  600s

//Oversampling. Number of averages. Part of OVS_CFG register.
typedef uint8_t Ens220_Oversampling;
#define ENS220_OVERSAMPLING_N_1     (0)                            //   1
#define ENS220_OVERSAMPLING_N_2     (1)                            //   2
#define ENS220_OVERSAMPLING_N_4     (2)                            //   4
#define ENS220_OVERSAMPLING_N_8     (3)                            //   8
#define ENS220_OVERSAMPLING_N_16    (4)                            //  16
#define ENS220_OVERSAMPLING_N_32    (5)                            //  32
#define ENS220_OVERSAMPLING_N_64    (6)                            //  64
#define ENS220_OVERSAMPLING_N_128   (7)                            // 128

//Moving average configuration. Part of MAVG_CFG register. Controls the number of samples used by the moving average filter (MAVG).
typedef uint8_t Ens220_MovingAverageSamples;
#define ENS220_MOVING_AVERAGE_SAMPLES_N_1     (0)                            //  1
#define ENS220_MOVING_AVERAGE_SAMPLES_N_2     (1)                            //  2
#define ENS220_MOVING_AVERAGE_SAMPLES_N_4     (2)                            //  4
#define ENS220_MOVING_AVERAGE_SAMPLES_N_8     (3)                            //  8
#define ENS220_MOVING_AVERAGE_SAMPLES_N_16    (4)                            // 16
#define ENS220_MOVING_AVERAGE_SAMPLES_N_32    (5)                            // 32

// Host interface configuration register (INTF_CFG).
typedef uint8_t Ens220_InterfaceConfiguration;
#define ENS220_INTERFACE_CONFIGURATION_INTERRUPT_ENABLE     (1 << 2)           // Interrupt enable
#define ENS220_INTERFACE_CONFIGURATION_INTERRUPT_POLARITY   (1 << 1)           // Interrupt polarity; 0b0: HIGH; 0b1: LOW
#define ENS220_INTERFACE_CONFIGURATION_SPI_3WIRE_MODE       (1 << 0)            // SPI mode control 0b1: SPI works in 3-wire mode

// Interrupt configuration register (INT_CFG).
typedef uint8_t Ens220_InterruptConfiguration;
#define ENS220_INTERRUPT_CONFIGURATION_TEMPERATURE_DATA_READY    (1 << 6)       // Interrupt is triggered when new temperature data becomes available
#define ENS220_INTERRUPT_CONFIGURATION_PRESSURE_FIFO_LEVEL_HIGH  (1 << 5)       // Interrupt is triggered when the filling level of the pressure FIFO becomes greater than the programmed threshold (FP_FILL_TH)
#define ENS220_INTERRUPT_CONFIGURATION_PRESSURE_FIFO_FULL        (1 << 4)       // Interrupt is triggered when the pressure FIFO becomes full (32 values)
#define ENS220_INTERRUPT_CONFIGURATION_PRESSURE_FIFO_EMPTY       (1 << 3)       // Interrupt is triggered when the pressure FIFO becomes empty
#define ENS220_INTERRUPT_CONFIGURATION_PRESSURE_DATA_READY       (1 << 2)       // interrupt is triggered when new pressure data becomes available
#define ENS220_INTERRUPT_CONFIGURATION_PRESSURE_HIGH             (1 << 1)       // Interrupt is triggered when the pressure is greater than the programmed pressure high threshold (PRESS_HI)
#define ENS220_INTERRUPT_CONFIGURATION_PRESSURE_LOW              (1 << 0)       // Interrupt is triggered when the pressure is lower than the programmed pressure low threshold (PRESS_LO)

// FIFO configuration register flags. Part of FIFO_CFG.
typedef uint8_t Ens220_FifoConfiguration;
#define ENS220_FIFOCONFIGURATION_CLEAR                          (1 << 5)                          // the content of the FIFO is cleared; FP_CLEAR is automatically cleared.

// Data status register (DATA_STAT).
typedef uint8_t Ens220_DataStatus;
#define ENS220_DATA_STATUS_PRESSURE_OVERWRITE       (1 << 3)       // This bit is set whenever a new pressure measurement is produced by the measurement engine while the pressure FIFO was already full.
#define ENS220_DATA_STATUS_TEMPERATURE_OVERWRITE    (1 << 2)       // This bit is set when a new temperature measurement is produced by the measurement engine and the previous data was not read.
#define ENS220_DATA_STATUS_PRESSURE_READY           (1 << 1)       // This bit is set when new pressure data becomes available.
#define ENS220_DATA_STATUS_TEMPERATURE_READY        (1 << 0)       // This bit is set when new temperature data becomes available.
#define ENS220_DATA_STATUS_NONE                     (0)

// FIFO status register (FIFO_STAT).
typedef uint8_t Ens220_FifoStatus;
#define ENS220_FIFO_STATUS_FULL                     (1 << 2)       // is set when the FIFO is enabled and is full (32 elements).
#define ENS220_FIFO_STATUS_EMPTY                    (1 << 1)       // is set when the FIFO is enabled and is empty (0 elements).
#define ENS220_FIFO_STATUS_REACHED_THRESHOLD        (1 << 0)       // is set when is enabled and the number of elements is greater than threshold (FP_FILL_TH).

// Interrupt status register (INT_STAT).
typedef uint8_t Ens220_InterruptStatus;
#define ENS220_INTERRUPT_STATUS_GENERAL                 (1 << 7)       // (IA) is set if any of the interrupt flags relative to sources enabled in INT_CFG are set;
#define ENS220_INTERRUPT_STATUS_TEMPERATURE             (1 << 6)       // (TR) is set when a new temperature measurement is produced by the measurement engine
#define ENS220_INTERRUPT_STATUS_FIFO_REACHED_THRESHOLD  (1 << 5)       // (FH) is set when FP_FILL > FP_FILL_TH.
#define ENS220_INTERRUPT_STATUS_FIFO_FULL               (1 << 4)       // (FF) is set when the FIFO becomes full.
#define ENS220_INTERRUPT_STATUS_FIFO_EMPTY              (1 << 3)       // (FE) is set when FIFO becomes empty as effect of a dataread.
#define ENS220_INTERRUPT_STATUS_PRESSURE                (1 << 2)       // (PR) is set when new pressure measurement is produced.
#define ENS220_INTERRUPT_STATUS_HIGH_PRESSURE           (1 << 1)       // (PH) is set when the measurement engine produces a pressure measurement that is greater than PRESS_HI.
#define ENS220_INTERRUPT_STATUS_LOW_PRESSURE            (1 << 0)       // (PL) is set when the measurement engine produces a pressure measurement that is lower than PRESS _LO
#define ENS220_INTERRUPT_STATUS_NONE                    (0)

// BufferInfo defines internally used buffer and data indices and sizes (in bytes)
#define ENS220_BUFFER_INFO_CONFIGURATION_SIZE   (uint8_t)(14)
#define ENS220_BUFFER_INFO_PRESSURE_SIZE        (uint8_t)(3)
#define ENS220_BUFFER_INFO_TEMPERATURE_SIZE     (uint8_t)(2)
#define ENS220_BUFFER_INFO_FIFO_SIZE            (uint8_t)(ENS220_BUFFER_INFO_PRESSURE_SIZE * 32)
#define ENS220_BUFFER_INFO_UPDATE_BUFFER_SIZE   (uint8_t)(ENS220_BUFFER_INFO_TEMPERATURE_SIZE + ENS220_BUFFER_INFO_PRESSURE_SIZE + ENS220_BUFFER_INFO_FIFO_SIZE)
#define ENS220_BUFFER_INFO_PRESSURE_INDEX       (uint8_t)(0)
#define ENS220_BUFFER_INFO_TEMPERATURE_INDEX    (uint8_t)(ENS220_BUFFER_INFO_PRESSURE_INDEX + ENS220_BUFFER_INFO_PRESSURE_SIZE)
#define ENS220_BUFFER_INFO_FIFO_INDEX           (uint8_t)(ENS220_BUFFER_INFO_TEMPERATURE_INDEX + ENS220_BUFFER_INFO_TEMPERATURE_SIZE)
#define ENS220_BUFFER_INFO_MODE_CFG             (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG    - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_MEAS_CFG             (uint8_t)ENS220_REGISTER_ADDRESS_MEAS_CFG    - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_STBY_CFG             (uint8_t)ENS220_REGISTER_ADDRESS_STBY_CFG    - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_OVS_CFG              (uint8_t)ENS220_REGISTER_ADDRESS_OVS_CFG     - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_MAVG_CFG             (uint8_t)ENS220_REGISTER_ADDRESS_MAVG_CFG    - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_INTF_CFG             (uint8_t)ENS220_REGISTER_ADDRESS_INTF_CFG    - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_INT_CFG              (uint8_t)ENS220_REGISTER_ADDRESS_INT_CFG     - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_PRESS_LO_XL          (uint8_t)ENS220_REGISTER_ADDRESS_PRESS_LO_XL - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_PRESS_LO_L           (uint8_t)ENS220_REGISTER_ADDRESS_PRESS_LO_L  - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_PRESS_LO_H           (uint8_t)ENS220_REGISTER_ADDRESS_PRESS_LO_H  - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_PRESS_HI_XL          (uint8_t)ENS220_REGISTER_ADDRESS_PRESS_HI_XL - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_PRESS_HI_L           (uint8_t)ENS220_REGISTER_ADDRESS_PRESS_HI_L  - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_PRESS_HI_H           (uint8_t)ENS220_REGISTER_ADDRESS_PRESS_HI_H  - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG
#define ENS220_BUFFER_INFO_FIFO_CFG             (uint8_t)ENS220_REGISTER_ADDRESS_FIFO_CFG    - (uint8_t)ENS220_REGISTER_ADDRESS_MODE_CFG

//// Result and Errors
#ifndef SCIOSENSE_RESULT_CODES
#define SCIOSENSE_RESULT_CODES
typedef int8_t Result;
#define RESULT_NOT_ALLOWED                                      (4)     // The requested command is not allowed.
#define RESULT_CHECKSUM_ERROR                                   (3)     // The value was read, but the checksum over the payload (valid and data) does not match.
#define RESULT_INVALID                                          (2)     // The value was read, but the data is invalid.
#define RESULT_IO_ERROR                                         (1)     // There was an IO communication error, read/write the stream failed.
#define RESULT_OK                                               (0)     // All OK; The value was read, the checksum matches, and data is valid.
#endif

#endif // SCIOSENSE_ENS220_DEFINES_C_H