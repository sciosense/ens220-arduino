#include "ens220.h"
#include "utils.h"

using namespace ScioSense::Utils;

namespace ScioSense
{
    ENS220::Result ENS220::update()
    {
        if (hasFlag((InterfaceConfiguration)configurationData[BufferInfo::INTF_CFG], InterfaceConfiguration::InterruptEnable))
        {
            return updateInterrupted();
        }
        else
        {
            return updatePolled();
        }
    }

    const uint8_t* ENS220::getRawConfiguration()
    {
        return configurationData;
    }

    uint8_t ENS220::getFifoThreshold()
    {
        return configurationData[BufferInfo::FIFO_CFG] & 0b00011111;
    }

    ENS220::PressureConversionTime ENS220::getPressureConversionTime()
    {
        return (PressureConversionTime)((configurationData[BufferInfo::MEAS_CFG] & 0b00011000) >> 3);
    }

    ENS220::Oversampling ENS220::getOversamplingOfPressure()
    {
        return (Oversampling)((configurationData[BufferInfo::OVS_CFG] & 0b00111000) >> 3);
    }

    ENS220::Oversampling ENS220::getOversamplingOfTemperature()
    {
        return (Oversampling)((configurationData[BufferInfo::OVS_CFG] & 0b00000111));
    }

    uint32_t ENS220::getUID()
    {
        return uid;
    }

    void ENS220::setDefaultConfiguration()
    {
        constexpr uint8_t defaults[] =
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

        memcpy(configurationData, defaults, sizeof(configurationData));
    }

    void ENS220::setRawConfiguration(const uint8_t* config)
    {
        memcpy(configurationData, config, sizeof(configurationData));
    };

    void ENS220::setPressureDataPath(const PressureDataPath& dataPath)
    {
        configurationData[BufferInfo::MODE_CFG] = (uint8_t)dataPath;
    }

    void ENS220::setPressureConversionTime(const PressureConversionTime& time)
    {
        configurationData[BufferInfo::MEAS_CFG] = (configurationData[BufferInfo::MEAS_CFG] & 0b00000111) | ((uint8_t)time << 3);
    };

    void ENS220::setPressureTemperatureRatio(const PressureTemperatureRatio& ratio)
    {
        configurationData[BufferInfo::MEAS_CFG] = (configurationData[BufferInfo::MEAS_CFG] & 0b00011000) | (uint8_t)ratio;
    };

    void ENS220::setStandbyTime(const StandbyTime& time)
    {
        configurationData[BufferInfo::STBY_CFG] = (uint8_t) time;
    };

    void ENS220::setOversamplingOfPressure(const Oversampling& n)
    {
        configurationData[BufferInfo::OVS_CFG] = (configurationData[BufferInfo::OVS_CFG] & 0b00000111) | ((uint8_t)n << 3);
    };

    void ENS220::setOversamplingOfTemperature (const Oversampling& n)
    {
        configurationData[BufferInfo::OVS_CFG] = (configurationData[BufferInfo::OVS_CFG] & 0b00111000) | (uint8_t)n;
    };

    void ENS220::setMovingAverageSamples(const MovingAverageSamples& n)
    {
        configurationData[BufferInfo::MAVG_CFG] = (uint8_t)n;
    };

    void ENS220::setInterfaceConfiguration(const InterfaceConfiguration& interfaceConf)
    {
        configurationData[BufferInfo::INTF_CFG] = (uint8_t)interfaceConf;
    };

    void ENS220::setInterruptConfiguration(const InterruptConfiguration& inerruptConf)
    {
        configurationData[BufferInfo::INT_CFG] = (uint8_t)inerruptConf;
    };

    void ENS220::setLowPressThreshold(const uint32_t& threshold)
    {
        configurationData[BufferInfo::PRESS_LO_XL]  = (threshold & 0x000000FF);
        configurationData[BufferInfo::PRESS_LO_L]   = (threshold & 0x0000FF00) >> 8;
        configurationData[BufferInfo::PRESS_LO_H]   = (threshold & 0x00FF0000) >> 16;
    };
    void ENS220::setHighPressThreshold(const uint32_t& threshold)
    {
        configurationData[BufferInfo::PRESS_HI_XL]  = (threshold & 0x000000FF);
        configurationData[BufferInfo::PRESS_HI_L]   = (threshold & 0x0000FF00) >> 8;
        configurationData[BufferInfo::PRESS_HI_H]   = (threshold & 0x00FF0000) >> 16;
    };

    void ENS220::setFifoThreshold(const uint8_t& threshold)
    {
        configurationData[BufferInfo::FIFO_CFG] = 0b00011111 & (uint8_t)threshold;
    };

    void ENS220::setInterruptPin(int p)
    {
        pinMode(p, INPUT);
        interruptPin = p;
    }

    inline uint16_t ENS220::getTempRaw() const
    {
        return endian::littleTo<uint16_t>(buffer + BufferInfo::TemperatureIndex);
    }

    float ENS220::getTempKelvin()
    {
        return (float)getTempRaw() / 128.0f;
    }

    float ENS220::getTempCelsius()
    {
        return getTempKelvin() - 273.15f;
    }

    float ENS220::getTempFahrenheit()
    {
        return (1.8f * getTempCelsius()) + 32.f;
    }

    float ENS220::getPressurePascal()
    {
        return (float)(endian::littleTo<uint32_t>(buffer) & 0x00ffffff) / 64.f;
    }

    float ENS220::getPressurePascal(uint8_t index)
    {
        return (float)(endian::littleTo<uint32_t>(buffer + BufferInfo::FifoIndex + (index * 3)) & 0x00ffffff) / 64.f;
    }

    float ENS220::getPressureHectoPascal()
    {
        return (getPressurePascal() / 100.f);
    }

    float ENS220::getPressureHectoPascal(uint8_t index)
    {
        return (getPressurePascal(index) / 100.f);
    }

    uint32_t ENS220::getPressureRaw()
    {
        return endian::littleTo<uint32_t>(buffer);
    }

    uint32_t ENS220::getPressureRaw(const uint8_t& fifoIndex) const
    {
        if (fifoIndex < 32)
        {
            return endian::littleTo<uint32_t>(buffer + BufferInfo::FifoIndex + (fifoIndex * 3)) & 0x00ffffff;
        }
        else
        {
            return endian::littleTo<uint32_t>(buffer) & 0x00ffffff;
        }
    }

    uint8_t* ENS220::getPressureBuffer()
    {
        return buffer + BufferInfo::FifoIndex;
    }

    ENS220::DataStatus ENS220::getDataStatus()
    {
        return dataStatus;
    }

    ENS220::InterruptStatus ENS220::getInterruptStatus()
    {
        return interruptStatus;
    }

    uint16_t ENS220::CalculateMeasurementTimeSingleShot(const Sensor& sensor)
    {
        uint16_t time;

        // calculate first temperature conversion time
        time = 3 + (1 << (uint8_t)getOversamplingOfTemperature());

        if(hasFlag((ModeConfiguration)sensor, ModeConfiguration::MeasurePressure))
        {
            uint16_t timeFirstPressConversionMs = (3 + (1 << (uint16_t)getOversamplingOfPressure())) * (1 << (uint16_t)getPressureConversionTime());
            time += timeFirstPressConversionMs;
        }

        // apply 20% + 1ms dispersion compensation
        time = ((time * 6) / 5) + 1;

        return time;
    }

    float ENS220::CalculateDryAirDensity(const uint8_t& fifoIndex) const
    {
        // this implements the following formula:
        // density = (p / 64.0) / ( (t / 128.0) * 287.0501 )

        constexpr uint64_t SPECIFIC_GAS_CONSTANT_DRY_AIR = 2870501;
        const uint32_t p                                 = getPressureRaw(fifoIndex);
        const uint16_t t                                 = getTempRaw();

        return p / ((SPECIFIC_GAS_CONSTANT_DRY_AIR * t) / 20000.f);
    }

    float ENS220::CalculateDryAirDensity(const ENS220& other, const uint8_t& fifoIndex) const
    {
        return (CalculateDryAirDensity(fifoIndex) + other.CalculateDryAirDensity(fifoIndex)) / 2.f;
    }

    float ENS220::CalculateHeightDifference(const ENS220& other, const uint8_t& fifoIndex) const
    {
        const float dryAirDensity                       = CalculateDryAirDensity(other, fifoIndex);
        const uint32_t p1                               = getPressureRaw(fifoIndex);
        const uint32_t p2                               = other.getPressureRaw(fifoIndex);

        return CalculateHeightDifference(dryAirDensity, p1, p2);
    }

    float ENS220::CalculateHeightDifference(const float& dryAirDensity, const ENS220& other, const uint8_t& fifoIndex) const
    {
        const uint32_t p1 = getPressureRaw(fifoIndex);
        const uint32_t p2 = other.getPressureRaw(fifoIndex);

        return CalculateHeightDifference(dryAirDensity, p1, p2);
    }

    float ENS220::CalculateHeightDifference(const float& dryAirDensity, const uint32_t& p1Raw, const uint32_t& p2Raw) const
    {
        // this implements the following formula:
        // h = ((p2 / 64.0) - (p1 / 64.0)) / (9.80665 * density)

        const float GRAVITATIONAL_ACCELERATION_SEA_LEVEL = 627.6256f; // 9.80665 * 64.f

        return ((int32_t)(p2Raw - p1Raw)) / (GRAVITATIONAL_ACCELERATION_SEA_LEVEL * dryAirDensity);
    }

    void ENS220::waitInterrupt()
    {
        if (interruptPin != 0)
        {
            if (hasFlag((InterfaceConfiguration)configurationData[BufferInfo::INTF_CFG], InterfaceConfiguration::InterruptPolarity))
            {
                while (digitalRead(interruptPin) != LOW); // wait until low
            }
            else
            {
                while (digitalRead(interruptPin) != HIGH); // wait until high
            }
        }
    }

    void ENS220::waitStandbyTime()
    {
        switch ((StandbyTime)configurationData[BufferInfo::STBY_CFG])
        {
            case StandbyTime::T_10:     delay(10/2);     break;
            case StandbyTime::T_20:     delay(20/2);     break;
            case StandbyTime::T_30:     delay(30/2);     break;
            case StandbyTime::T_50:     delay(50/2);     break;
            case StandbyTime::T_100:    delay(100/2);    break;
            case StandbyTime::T_250:    delay(250/2);    break;
            case StandbyTime::T_500:    delay(500/2);    break;
            case StandbyTime::T_750:    delay(750/2);    break;
            case StandbyTime::T_1000:   delay(1000/2);   break;
            case StandbyTime::T_2000:   delay(2000/2);   break;
            case StandbyTime::T_5000:   delay(5000/2);   break;
            case StandbyTime::T_10000:  delay(10000/2);  break;
            case StandbyTime::T_60000:  delay(60000/2);  break;
            case StandbyTime::T_600000: delay(600000/2); break;

            default: break;
        }
    }

    void ENS220::waitSingleShot()
    {
        delay(CalculateMeasurementTimeSingleShot((Sensor)modeConfiguration));
    }

    ENS220::Result ENS220::read(RegisterAddress address, uint8_t* data, size_t size)
    {
        if (ioInterface)
        {
            return ioInterface->read(address, data, size);
        }

        return Result::IOError;
    }

    ENS220::Result ENS220::write(RegisterAddress address, uint8_t* data, size_t size)
    {
        if (ioInterface)
        {
            return ioInterface->write(address, data, size);
        }

        return Result::IOError;
    }

    void ENS220::debug(const char* msg)
    {
        if (debugStream)
        {
            debugStream->print(debugPrefix);
            debugStream->println(msg);
        }
    }

    void ENS220::debug(const char* msg, Result& result)
    {
        debug(msg, nullptr, 0, result);
    }

    void ENS220::debug(const char* msg, uint8_t* data, size_t size, Result& result)
    {
        if (debugStream)
        {
            debugStream->print(debugPrefix);
            debugStream->print(msg);

            for (size_t i = 0; i < size; i++)
            {
                debugStream->print(" 0x");
                debugStream->print(data[i], HEX);
            }

            debugStream->print(" status: ");
            debugStream->println(toString(result));
        }
    }

    template<class T>
    void ENS220::debug(const char* msg, T data)
    {
        if (debugStream)
        {
            debugStream->print(debugPrefix);
            debugStream->print(msg);
            debugStream->print(" 0x");
            debugStream->print(data, HEX);
            debugStream->println();
        }
    }

    const char* ENS220::toString(const ENS220::Result& result)
    {
        switch (result)
        {
            case Result::NotAllowed     : return "not-allowed";
            case Result::IOError        : return "io-error";
            case Result::ChecksumError  : return "checksum-error";
            case Result::Invalid        : return "data-invalid";
            case Result::Ok             : return "ok";
            default                     : return "unknown-status";
        }
    }
}