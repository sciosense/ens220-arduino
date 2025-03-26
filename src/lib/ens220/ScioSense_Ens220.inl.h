#ifndef SCIOSENSE_ENS220_INL_C_H
#define SCIOSENSE_ENS220_INL_C_H

#include "ScioSense_Ens220.h"

#define wait(ms)            ens220->io.wait(ms)
#define hasAnyFlag(a, b)    (((a) & (b)) != 0)
#define hasFlag(a, b)       (((a) & (b)) == (b))
#define memcpy(a, b, s)     for(size_t i = 0; i < s; i++) {a[i] = b[i];}

static inline uint16_t Ens220_GetValueOf16(const uint8_t* data, const uint16_t resultAddress)
{
    return ((uint16_t)data[resultAddress + 1] << 8) + (uint16_t)data[resultAddress];
}

static inline uint32_t Ens220_GetValueOf32(const uint8_t* data, const uint16_t resultAddress)
{
    return  ((uint32_t)data[resultAddress + 3] << 24)
          + ((uint32_t)data[resultAddress + 2] << 16)
          + ((uint32_t)data[resultAddress + 1] <<  8)
          +  (uint32_t)data[resultAddress + 0];
}

static inline Result Ens220_Read(ScioSense_Ens220* ens220, const uint16_t address, uint8_t* data, const size_t size)
{
    return ens220->io.read(ens220->io.config, address, data, size);
}

static inline Result Ens220_Write(ScioSense_Ens220* ens220, const uint16_t address, uint8_t* data, const size_t size)
{
    return ens220->io.write(ens220->io.config, address, data, size);
}

static inline Result Ens220_Init(ScioSense_Ens220* ens220)
{
    Result result;

    result = Ens220_Reset(ens220);
    if (result != RESULT_OK)
    {
        return result;
    }
    
    Ens220_SetHighPower(ens220, true);
    {
        result = Ens220_ReadPartId(ens220);
        if (result != RESULT_OK)
        {
            return result;
        }

        result = Ens220_ReadUid(ens220);
        if (result != RESULT_OK)
        {
            return result;
        }
    }
    Ens220_SetHighPower(ens220, false);

    return result;
}

static inline Result Ens220_Update(ScioSense_Ens220* ens220)
{
    if (hasFlag((Ens220_InterfaceConfiguration)ens220->configurationBuffer[ENS220_BUFFER_INFO_INTF_CFG], ENS220_INTERFACE_CONFIGURATION_INTERRUPT_ENABLE))
    {
        return Ens220_UpdateInterrupted(ens220);
    }
    else
    {
        return Ens220_UpdatePolled(ens220);
    }
}

static inline Result Ens220_UpdateInterrupted(ScioSense_Ens220* ens220)
{
    Result result;

    result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_INT_STAT, &ens220->interruptStatus, sizeof(ens220->interruptStatus));
    if (result == RESULT_OK)
    {
        if (hasFlag(ens220->interruptStatus, ENS220_INTERRUPT_STATUS_TEMPERATURE))
        {
            result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_TEMP_OUT_L, ens220->dataBuffer + ENS220_BUFFER_INFO_TEMPERATURE_INDEX, ENS220_BUFFER_INFO_TEMPERATURE_SIZE);
        }

        if (result == RESULT_OK)
        {
            if (hasFlag(ens220->interruptStatus, ENS220_INTERRUPT_STATUS_FIFO_FULL))
            {
                result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_PRESS_OUT_F_XL, ens220->dataBuffer + ENS220_BUFFER_INFO_FIFO_INDEX, ENS220_BUFFER_INFO_FIFO_SIZE);
            }
            else if (hasFlag(ens220->interruptStatus, ENS220_INTERRUPT_STATUS_FIFO_REACHED_THRESHOLD))
            {
                result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_PRESS_OUT_F_XL, ens220->dataBuffer + ENS220_BUFFER_INFO_FIFO_INDEX, Ens220_GetFifoThreshold(ens220) * ENS220_BUFFER_INFO_PRESSURE_SIZE);
            }
            else if (hasFlag(ens220->interruptStatus, ENS220_INTERRUPT_STATUS_PRESSURE))
            {
                result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_PRESS_OUT_XL, ens220->dataBuffer, ENS220_BUFFER_INFO_PRESSURE_SIZE);
            }
        }
    }

    return result;
}

static inline Result Ens220_UpdatePolled(ScioSense_Ens220* ens220)
{
    Result result;

    result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_DATA_STAT, &ens220->dataStatus, sizeof(ens220->dataStatus));
    if (result == RESULT_OK)
    {

        result = RESULT_INVALID;

        if (    hasFlag(ens220->modeConfiguration, ENS220_MODE_CONFIGURATION_MEASURE_PRESSURE)
            &&  hasFlag(ens220->dataStatus, ENS220_DATA_STATUS_PRESSURE_READY))
        {
            result = RESULT_OK;
        }

        if (    hasFlag(ens220->modeConfiguration, ENS220_MODE_CONFIGURATION_MEASURE_TEMPERATURE)
            &&  hasFlag(ens220->dataStatus, ENS220_DATA_STATUS_TEMPERATURE_READY))
        {
            result = RESULT_OK;
        }

        if (result == RESULT_OK)
        {
            result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_PRESS_OUT_XL, ens220->dataBuffer, ENS220_BUFFER_INFO_PRESSURE_SIZE + ENS220_BUFFER_INFO_TEMPERATURE_SIZE);
        }
    }

    return result;
}

static inline Result Ens220_ReadPartId(ScioSense_Ens220* ens220)
{
    Result result;
    uint8_t data[2];

    result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_PART_ID, data, 2);
    if (result == RESULT_OK)
    {
        ens220->partId = Ens220_GetValueOf16(data, 0);
    }

    return result;
}

static inline Result Ens220_ReadUid(ScioSense_Ens220* ens220)
{
    Result result;
    uint8_t data[4];

    result = Ens220_Read(ens220, ENS220_REGISTER_ADDRESS_UID, data, sizeof(data));
    if (result == RESULT_OK)
    {
        ens220->uid = Ens220_GetValueOf32(data, 0);
    }

    return result;
}

static inline Result Ens220_SetupSingleShotMeasure(ScioSense_Ens220* ens220, Ens220_Sensor sensor)
{
    Result result                       = RESULT_OK;
    Ens220_StandbyTime oneShotOperation = ENS220_STANDBY_TIME_ONE_SHOT_OPERATION;

    if (ens220->standbyTime != oneShotOperation)
    {
        Ens220_SetHighPower(ens220, true);
        {
            result = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_STBY_CFG, &oneShotOperation, sizeof(oneShotOperation));
        }
        Ens220_SetHighPower(ens220, false);

        if (result == RESULT_OK)
        {
            ens220->standbyTime = oneShotOperation;
        }
    }

    ens220->modeConfiguration  = ((Ens220_ModeConfiguration)ens220->configurationBuffer[ENS220_BUFFER_INFO_MODE_CFG] & ~ENS220_MODE_CONFIGURATION_MASK);
    ens220->modeConfiguration |=  (Ens220_ModeConfiguration)sensor;
    ens220->modeConfiguration |=  ENS220_MODE_CONFIGURATION_START;

    return result;
}

static inline Result Ens220_SingleShotMeasure(ScioSense_Ens220* ens220, Ens220_Sensor sensor, ScioSense_Ens220* other)
{
    Result result1, result2;

    Ens220_SetupSingleShotMeasure(ens220, sensor);

    if (other)
    {
        Ens220_SetupSingleShotMeasure(other, sensor);

        result1 = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &ens220->modeConfiguration, sizeof(ens220->modeConfiguration));
        result2 = Ens220_Write(other,  ENS220_REGISTER_ADDRESS_MODE_CFG, &ens220->modeConfiguration, sizeof(ens220->modeConfiguration));

        if (result2 != RESULT_OK)
        {
            result1 = result2;
        }
    }
    else
    {
        result1 = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &ens220->modeConfiguration, sizeof(ens220->modeConfiguration));
    }

    return result1;
}

static inline Result Ens220_StartContinuousMeasure(ScioSense_Ens220* ens220, Ens220_Sensor sensor, ScioSense_Ens220* other)
{
    ens220->modeConfiguration   = ((Ens220_ModeConfiguration)ens220->configurationBuffer[ENS220_BUFFER_INFO_MODE_CFG] & ~ENS220_MODE_CONFIGURATION_MASK);
    ens220->modeConfiguration  |= (Ens220_ModeConfiguration)sensor;
    ens220->modeConfiguration  |= ENS220_MODE_CONFIGURATION_HIGH_POWER;
    ens220->modeConfiguration  |= ENS220_MODE_CONFIGURATION_START;

    Result result1, result2;
    if (other)
    {
        result1 = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &ens220->modeConfiguration, sizeof(ens220->modeConfiguration));
        result2 = Ens220_Write(other,  ENS220_REGISTER_ADDRESS_MODE_CFG, &ens220->modeConfiguration, sizeof(ens220->modeConfiguration));

        if(result2 != RESULT_OK)
        {
            result1 = result2;
        }
    }
    else
    {
        result1 = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &ens220->modeConfiguration, sizeof(ens220->modeConfiguration));
    }

    return result1;
}

static inline Result Ens220_StopContinuousMeasure(ScioSense_Ens220* ens220)
{
    return Ens220_Reset(ens220);
}

static inline Result Ens220_SetHighPower(ScioSense_Ens220* ens220, bool enable)
{
    Result result;
    Ens220_ModeConfiguration conf;

    if (enable)
    {
        conf = ENS220_MODE_CONFIGURATION_HIGH_POWER;
    }
    else
    {
        conf = 0;
    }
    result = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &conf, sizeof(conf));

    wait(1);
    return result;
}

static inline Result Ens220_Reset(ScioSense_Ens220* ens220)
{
    Result result;
    uint8_t data;

    data    = ENS220_MODE_CONFIGURATION_RESET;
    result  = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &data, sizeof(data));
    if (result == RESULT_OK)
    {
        wait(1);

        if (ens220->io.spi3WireMode)
        {
            data    = ENS220_INTERFACE_CONFIGURATION_SPI_3WIRE_MODE;
            result  = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_INTF_CFG, &data, sizeof(data));
        }

        data    = 0;
        result  = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MODE_CFG, &data, sizeof(data));
    }

    return result;
}

static inline bool Ens220_IsConnected(const ScioSense_Ens220* ens220)
{
    return ens220->partId >= 0x0320 && ens220->partId < 0x0330;
}

static inline uint16_t Ens220_GetTempRaw(const ScioSense_Ens220* ens220)
{
    return Ens220_GetValueOf16(ens220->dataBuffer, ENS220_BUFFER_INFO_TEMPERATURE_INDEX);
}

static inline float Ens220_GetTempKelvin(const ScioSense_Ens220* ens220)
{
    return (float)Ens220_GetTempRaw(ens220) / 128.0f;
}

static inline float Ens220_GetTempCelsius(const ScioSense_Ens220* ens220)
{
    return Ens220_GetTempKelvin(ens220) - 273.15f;
}

static inline float Ens220_GetTempFahrenheit(const ScioSense_Ens220* ens220)
{
    return (1.8f * Ens220_GetTempCelsius(ens220)) + 32.f;
}

static inline uint32_t Ens220_GetPressureRaw(const ScioSense_Ens220* ens220)
{
    return Ens220_GetValueOf32(ens220->dataBuffer, 0) & 0x00ffffff;
}

static inline uint32_t Ens220_GetPressureRawFifo(const ScioSense_Ens220* ens220, const uint8_t fifoIndex)
{
    if (fifoIndex < 32)
    {
        return Ens220_GetValueOf32(ens220->dataBuffer, ENS220_BUFFER_INFO_FIFO_INDEX + (fifoIndex * 3)) & (uint32_t)0x00ffffff;
    }
    else
    {
        return Ens220_GetPressureRaw(ens220);
    }
}

static inline float Ens220_GetPressurePascal(const ScioSense_Ens220* ens220)
{
    return (float)Ens220_GetPressureRaw(ens220) / 64.f;
}

static inline float Ens220_GetPressurePascalFifo(const ScioSense_Ens220* ens220, uint8_t fifoIndex)
{
    return (float)Ens220_GetPressureRawFifo(ens220, fifoIndex) / 64.f;
}

static inline float Ens220_GetPressureHectoPascal(const ScioSense_Ens220* ens220)
{
    return (Ens220_GetPressurePascal(ens220) / 100.f);
}

static inline float Ens220_GetPressureHectoPascalFifo(const ScioSense_Ens220* ens220, uint8_t fifoIndex)
{
    return (Ens220_GetPressurePascalFifo(ens220, fifoIndex) / 100.f);
}

static inline const uint8_t* Ens220_GetPressureBuffer(const ScioSense_Ens220* ens220)
{
    return ens220->dataBuffer + ENS220_BUFFER_INFO_FIFO_INDEX;
}

static inline Ens220_DataStatus Ens220_GetDataStatus(const ScioSense_Ens220* ens220)
{
    return ens220->dataStatus;
}

static inline Ens220_InterruptStatus Ens220_GetInterruptStatus(const ScioSense_Ens220* ens220)
{
    return ens220->interruptStatus;
}

static inline const uint8_t* Ens220_GetRawConfiguration(const ScioSense_Ens220* ens220)
{
    return ens220->configurationBuffer;
}

static inline uint8_t Ens220_GetFifoThreshold(const ScioSense_Ens220* ens220)
{
    static const uint8_t mask = 0x1F; // 0b00011111
    return ens220->configurationBuffer[ENS220_BUFFER_INFO_FIFO_CFG] & mask;
}

static inline Ens220_PressureConversionTime Ens220_GetPressureConversionTime(const ScioSense_Ens220* ens220)
{
    static const uint8_t mask = 0x18; // 0b00011000
    return (Ens220_PressureConversionTime)((ens220->configurationBuffer[ENS220_BUFFER_INFO_MEAS_CFG] & mask) >> 3);
}

static inline Ens220_Oversampling Ens220_GetOversamplingOfPressure(const ScioSense_Ens220* ens220)
{
    static const uint8_t mask = 0x38; // 0b00111000
    return (Ens220_Oversampling)((ens220->configurationBuffer[ENS220_BUFFER_INFO_OVS_CFG] & mask) >> 3);
}

static inline Ens220_Oversampling Ens220_GetOversamplingOfTemperature(const ScioSense_Ens220* ens220)
{
    static const uint8_t mask = 7; // 0b00000111
    return (Ens220_Oversampling)((ens220->configurationBuffer[ENS220_BUFFER_INFO_OVS_CFG] & mask));
}

static inline uint32_t Ens220_GetUid(const ScioSense_Ens220* ens220)
{
    return ens220->uid;
}

static inline void Ens220_SetDefaultConfiguration(ScioSense_Ens220* ens220)
{
    const uint8_t defaults[] =
    {
        0x00, //MODE_CFG
        0x08, //MEAS_CFG
        0x00, //STBY_CFG
        0x00, //OVS_CFG
        0x00, //MAVG_CFG
        0x00, //INTF_CFG
        0x7F, //INT_CFG
        0x00, //PRESS_LO_TH
        0x00, //PRESS_LO_TH
        0x00, //PRESS_LO_TH
        0xff, //PRESS_HI_TH
        0xff, //PRESS_HI_TH
        0xff, //PRESS_HI_TH
        0x1f, //FIFO_CFG
    };

    memcpy(ens220->configurationBuffer, defaults, sizeof(ens220->configurationBuffer));
}

static inline void Ens220_SetRawConfiguration(ScioSense_Ens220* ens220, const uint8_t* config)
{
    memcpy(ens220->configurationBuffer, config, sizeof(ens220->configurationBuffer));
}

static inline void Ens220_SetPressureDataPath(ScioSense_Ens220* ens220, const Ens220_PressureDataPath dataPath)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_MODE_CFG] = (uint8_t)dataPath;
}

static inline void Ens220_SetPressureConversionTime(ScioSense_Ens220* ens220, const Ens220_PressureConversionTime time)
{
    static const uint8_t mask = 7; // 0b00000111
    ens220->configurationBuffer[ENS220_BUFFER_INFO_MEAS_CFG] = (ens220->configurationBuffer[ENS220_BUFFER_INFO_MEAS_CFG] & mask) | ((uint8_t)time << 3);
}

static inline void Ens220_SetPressureTemperatureRatio(ScioSense_Ens220* ens220, const Ens220_PressureTemperatureRatio ratio)
{
    static const uint8_t mask = 0x18; // 0b00011000
    ens220->configurationBuffer[ENS220_BUFFER_INFO_MEAS_CFG] = (ens220->configurationBuffer[ENS220_BUFFER_INFO_MEAS_CFG] & mask) | (uint8_t)ratio;
}

static inline void Ens220_SetStandbyTime(ScioSense_Ens220* ens220, const Ens220_StandbyTime time)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_STBY_CFG] = (uint8_t)time;
}

static inline void Ens220_SetOversamplingOfPressure(ScioSense_Ens220* ens220, const Ens220_Oversampling n)
{
    static const uint8_t mask = 7; // 0b00000111
    ens220->configurationBuffer[ENS220_BUFFER_INFO_OVS_CFG] = (ens220->configurationBuffer[ENS220_BUFFER_INFO_OVS_CFG] & mask) | ((uint8_t)n << 3);
}

static inline void Ens220_SetOversamplingOfTemperature(ScioSense_Ens220* ens220, const Ens220_Oversampling n)
{
    static const uint8_t mask = 0x38; // 0b00111000
    ens220->configurationBuffer[ENS220_BUFFER_INFO_OVS_CFG] = (ens220->configurationBuffer[ENS220_BUFFER_INFO_OVS_CFG] & mask) | (uint8_t)n;
}

static inline void Ens220_SetMovingAverageSamples(ScioSense_Ens220* ens220, const Ens220_MovingAverageSamples n)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_MAVG_CFG] = (uint8_t)n;
}

static inline void Ens220_SetInterfaceConfiguration(ScioSense_Ens220* ens220, const Ens220_InterfaceConfiguration interfaceConf)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_INTF_CFG] = (uint8_t)interfaceConf;
}

static inline void Ens220_SetInterruptConfiguration(ScioSense_Ens220* ens220, const Ens220_InterruptConfiguration interruptConf)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_INT_CFG] = (uint8_t)interruptConf;
}

static inline void Ens220_SetLowPressThreshold(ScioSense_Ens220* ens220, const uint32_t threshold)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_PRESS_LO_XL]  = (threshold & 0x000000FF);
    ens220->configurationBuffer[ENS220_BUFFER_INFO_PRESS_LO_L]   = (threshold & 0x0000FF00) >> 8;
    ens220->configurationBuffer[ENS220_BUFFER_INFO_PRESS_LO_H]   = (threshold & 0x00FF0000) >> 16;
}

static inline void Ens220_SetHighPressThreshold(ScioSense_Ens220* ens220, const uint32_t threshold)
{
    ens220->configurationBuffer[ENS220_BUFFER_INFO_PRESS_HI_XL]  = (threshold & 0x000000FF);
    ens220->configurationBuffer[ENS220_BUFFER_INFO_PRESS_HI_L]   = (threshold & 0x0000FF00) >> 8;
    ens220->configurationBuffer[ENS220_BUFFER_INFO_PRESS_HI_H]   = (threshold & 0x00FF0000) >> 16;
}

static inline void Ens220_SetFifoThreshold(ScioSense_Ens220* ens220, const uint8_t threshold)
{
    static const uint8_t mask = 0x1F; // 0b00011111
    ens220->configurationBuffer[ENS220_BUFFER_INFO_FIFO_CFG] = mask & (uint8_t)threshold;
}

static inline Result Ens220_WriteConfiguration(ScioSense_Ens220* ens220)
{
    Result result;

    Ens220_Reset(ens220);

    if (ens220->io.spi3WireMode)
    {
        ens220->configurationBuffer[ENS220_BUFFER_INFO_INTF_CFG] |= (uint8_t)ENS220_INTERFACE_CONFIGURATION_SPI_3WIRE_MODE;
    }

    Ens220_SetHighPower(ens220, true);
    {
        result = Ens220_Write(ens220, ENS220_REGISTER_ADDRESS_MEAS_CFG, ens220->configurationBuffer + ENS220_BUFFER_INFO_MEAS_CFG, sizeof(ens220->configurationBuffer) - 1);
    }
    Ens220_SetHighPower(ens220, false);
    
    ens220->standbyTime = (Ens220_StandbyTime)ens220->configurationBuffer[ENS220_BUFFER_INFO_STBY_CFG];

    return result;
}

static inline void Ens220_WaitStandbyTime(const ScioSense_Ens220* ens220)
{
    switch ((Ens220_StandbyTime)ens220->configurationBuffer[ENS220_BUFFER_INFO_STBY_CFG])
    {
        case ENS220_STANDBY_TIME_T_10:     wait(10 / 2);     break;
        case ENS220_STANDBY_TIME_T_20:     wait(20 / 2);     break;
        case ENS220_STANDBY_TIME_T_30:     wait(30 / 2);     break;
        case ENS220_STANDBY_TIME_T_50:     wait(50 / 2);     break;
        case ENS220_STANDBY_TIME_T_100:    wait(100 / 2);    break;
        case ENS220_STANDBY_TIME_T_250:    wait(250 / 2);    break;
        case ENS220_STANDBY_TIME_T_500:    wait(500 / 2);    break;
        case ENS220_STANDBY_TIME_T_750:    wait(750 / 2);    break;
        case ENS220_STANDBY_TIME_T_1000:   wait(1000 / 2);   break;
        case ENS220_STANDBY_TIME_T_2000:   wait(2000 / 2);   break;
        case ENS220_STANDBY_TIME_T_5000:   wait(5000 / 2);   break;
        case ENS220_STANDBY_TIME_T_10000:  wait(10000 / 2);  break;
        case ENS220_STANDBY_TIME_T_60000:  wait(60000 / 2);  break;
        case ENS220_STANDBY_TIME_T_600000: wait(600000 / 2); break;

        default: break;
    }
}

static inline void Ens220_WaitSingleShot(const ScioSense_Ens220* ens220)
{
    wait(Ens220_CalculateMeasurementTimeSingleShot(ens220, (Ens220_Sensor)ens220->modeConfiguration));
}

static inline uint16_t Ens220_CalculateMeasurementTimeSingleShot(const ScioSense_Ens220* ens220, const Ens220_Sensor sensor)
{
    uint16_t time;

    // calculate first temperature conversion time
    time = 3 + (1 << (uint8_t)Ens220_GetOversamplingOfTemperature(ens220));

    if(hasFlag(sensor, ENS220_MODE_CONFIGURATION_MEASURE_PRESSURE))
    {
        uint16_t timeFirstPressConversionMs = (3 + (1 << (uint16_t)Ens220_GetOversamplingOfPressure(ens220))) * (1 << (uint16_t)Ens220_GetPressureConversionTime(ens220));
        time += timeFirstPressConversionMs;
    }

    // apply 20% + 1ms dispersion compensation
    time = ((time * 6) / 5) + 1;

    return time;
}

static inline float Ens220_CalculateDryAirDensity(const ScioSense_Ens220* ens220, const uint8_t fifoIndex)
{
    // this implements the following formula:
    // density = (p / 64.0) / ( (t / 128.0) * 287.0501 )

    static const uint64_t SPECIFIC_GAS_CONSTANT_DRY_AIR = 2870501;
    const uint32_t p                                    = Ens220_GetPressureRawFifo(ens220, fifoIndex);
    const uint16_t t                                    = Ens220_GetTempRaw(ens220);

    return p / ((SPECIFIC_GAS_CONSTANT_DRY_AIR * t) / 20000.f);
}

static inline float Ens220_CalculateDryAirDensity2(const ScioSense_Ens220* ens220, const ScioSense_Ens220* other, const uint8_t fifoIndex)
{
    return (Ens220_CalculateDryAirDensity(ens220, fifoIndex) + Ens220_CalculateDryAirDensity(other, fifoIndex)) / 2.f;
}

static inline float Ens220_CalculateHeightDifference(const ScioSense_Ens220* ens220, const ScioSense_Ens220* other, const uint8_t fifoIndex)
{
    const float dryAirDensity                       = Ens220_CalculateDryAirDensity(other, fifoIndex);
    const uint32_t p1                               = Ens220_GetPressureRawFifo(ens220, fifoIndex);
    const uint32_t p2                               = Ens220_GetPressureRawFifo(other,  fifoIndex);

    return Ens220_CalculateHeightDifference3(dryAirDensity, p1, p2);
}

static inline float Ens220_CalculateHeightDifference2(const ScioSense_Ens220* ens220, const float dryAirDensity, const ScioSense_Ens220* other, const uint8_t fifoIndex)
{
    const uint32_t p1 = Ens220_GetPressureRawFifo(ens220, fifoIndex);
    const uint32_t p2 = Ens220_GetPressureRawFifo(other,  fifoIndex);

    return Ens220_CalculateHeightDifference3(dryAirDensity, p1, p2);
}

static inline float Ens220_CalculateHeightDifference3(const float dryAirDensity, const uint32_t p1Raw, const uint32_t p2Raw)
{
    // this implements the following formula:
    // h = ((p2 / 64.0) - (p1 / 64.0)) / (9.80665 * density)

    const float GRAVITATIONAL_ACCELERATION_SEA_LEVEL = 627.6256f; // 9.80665 * 64.f

    return ((int32_t)(p2Raw - p1Raw)) / (GRAVITATIONAL_ACCELERATION_SEA_LEVEL * dryAirDensity);
}


#undef wait
#undef hasAnyFlag
#undef hasFlag
#undef memcpy

#endif // SCIOSENSE_ENS21X_INL_C_H