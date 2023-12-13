#include <Arduino.h>
#include <Wire.h>

#include "ens220.h"
#include "utils.h"

using namespace ScioSense::Utils;

namespace ScioSense
{
    ENS220::~ENS220() { }

    ENS220::ENS220()
    {
        debugStream         = nullptr;
        interruptPin        = NOT_A_PIN;

        standbyTime         = StandbyTime::ContinousOperation;
        dataStatus          = DataStatus::None;
        interruptStatus     = InterruptStatus::None;

        setDefaultConfiguration();
    }

    bool ENS220::isConnected()
    {
        return partId >= 0x0320 &&  partId < 0x0330;
    }

    bool ENS220::begin(IoInterface<RegisterAddress, Result>* ioInterface, bool spi3WireMode)
    {
        this->ioInterface  = ioInterface;
        this->spi3WireMode = spi3WireMode;

        if (spi3WireMode)
        {
            write(RegisterAddress::INTF_CFG, InterfaceConfiguration::Spi3WireMode);
        }

        read(RegisterAddress::PART_ID, partId);
        debug("PART_ID  : ", partId);

        if (isConnected())
        {
            setHighPower(true);
            read(RegisterAddress::UID, uid);
            debug("UID      : ", uid);
            setHighPower(false);
        }

        return isConnected();
    }

    ENS220::Result ENS220::reset()
    {
        Result result = write(RegisterAddress::MODE_CFG, ModeConfiguration::Reset);
        if (result == Result::Ok)
        {
            delayMicroseconds(500);

            if (spi3WireMode)
            {
                write(RegisterAddress::INTF_CFG, InterfaceConfiguration::Spi3WireMode);
            }
            result = write(RegisterAddress::MODE_CFG, (uint8_t)0);
        }

        debug(__func__, result);
        return result;
    }

    ENS220::Result ENS220::setHighPower(bool enable)
    {
        Result result;

        if (enable)
        {
            result = write(RegisterAddress::MODE_CFG,ModeConfiguration::HighPower);
        }
        else
        {
            result = write(RegisterAddress::MODE_CFG, (uint8_t)0);
        }

        delayMicroseconds(500);

        debug(__func__, result);
        return result;
    }

    void ENS220::writeConfiguration()
    {
        reset();

        if(spi3WireMode)
        {
            configurationData[BufferInfo::INTF_CFG] |= (uint8_t)InterfaceConfiguration::Spi3WireMode;
        }

        setHighPower(true);
        write(RegisterAddress::MEAS_CFG, configurationData + BufferInfo::MEAS_CFG, sizeof(configurationData) - 1);
        setHighPower(false);

        standbyTime = (StandbyTime)configurationData[BufferInfo::STBY_CFG];
    }

    ENS220::Result ENS220::updateInterrupted()
    {
        Result result;

        result = read(RegisterAddress::INT_STAT, interruptStatus);
        if (result == Result::Ok)
        {
            if (hasFlag(interruptStatus, InterruptStatus::Temperature))
            {
                result = read(RegisterAddress::TEMP_OUT_L, buffer + BufferInfo::TemperatureIndex, BufferInfo::TemperatureSize);
            }

            if (result == Result::Ok)
            {
                if (hasFlag(interruptStatus, InterruptStatus::FifoFull))
                {
                    result = read(RegisterAddress::PRESS_OUT_F_XL, buffer + BufferInfo::FifoIndex, BufferInfo::FifoSize);
                }
                else if (hasFlag(interruptStatus, InterruptStatus::FifoReachedThreshold))
                {
                    result = read(RegisterAddress::PRESS_OUT_F_XL, buffer + BufferInfo::FifoIndex, getFifoThreshold() * BufferInfo::PressureSize);
                }
                else if (hasFlag(interruptStatus, InterruptStatus::Pressure))
                {
                    result = read(RegisterAddress::PRESS_OUT_XL, buffer, BufferInfo::PressureSize);
                }
            }
        }

        debug(__func__, result);
        return result;
    }
    ENS220::Result ENS220::updatePolled()
    {
        Result result;

        result = read(RegisterAddress::DATA_STAT, dataStatus);
        if (result == Result::Ok)
        {

            result = Result::Invalid;

            if (hasFlag(modeConfiguration, ModeConfiguration::MeasurePressure) && hasFlag(dataStatus, DataStatus::PressureReady))
            {
                result = Result::Ok;
            }

            if (hasFlag(modeConfiguration, ModeConfiguration::MeasureTemperature) && hasFlag(dataStatus, DataStatus::TemperatureReady))
            {
                result = Result::Ok;
            }

            if (result == Result::Ok)
            {
                result = read(RegisterAddress::PRESS_OUT_XL, buffer, BufferInfo::PressureSize + BufferInfo::TemperatureSize);
            }

        }

        debug(__func__, result);
        return result;
    }

    ENS220::Result ENS220::setupSingleShotMeasure(Sensor sensor)
    {
        Result result = Result::Ok;

        if (standbyTime != StandbyTime::OneShotOperation)
        {
            setHighPower(true);
            result = write(RegisterAddress::STBY_CFG, StandbyTime::OneShotOperation);
            setHighPower(false);

            if (result == Result::Ok)
            {
                standbyTime = StandbyTime::OneShotOperation;
            }
        }

        modeConfiguration   = ((ModeConfiguration)configurationData[BufferInfo::MODE_CFG] & ~ModeConfiguration::Mask);
        modeConfiguration  |= (ModeConfiguration)sensor;
        modeConfiguration  |= ModeConfiguration::Start;

        return result;
    }

    ENS220::Result ENS220::singleShotMeasure(Sensor sensor, ENS220* other)
    {
        Result result1, result2;

        setupSingleShotMeasure(sensor);

        if(other)
        {
            other->setupSingleShotMeasure(sensor);

            result1 =        write(RegisterAddress::MODE_CFG, modeConfiguration);
            result2 = other->write(RegisterAddress::MODE_CFG, modeConfiguration);

            if(result2 != Result::Ok)
            {
                result1 = result2;
            }
        }
        else
        {
            result1 = write(RegisterAddress::MODE_CFG, modeConfiguration);
        }

        debug(__func__, result1);
        return result1;
    }

    ENS220::Result ENS220::startContinuousMeasure(Sensor sensor, ENS220* other)
    {
        modeConfiguration   = ((ModeConfiguration)configurationData[BufferInfo::MODE_CFG] & ~ModeConfiguration::Mask);
        modeConfiguration  |= (ModeConfiguration)sensor;
        modeConfiguration  |= ModeConfiguration::HighPower;
        modeConfiguration  |= ModeConfiguration::Start;

        Result result1, result2;
        if (other)
        {
            result1 =        write(RegisterAddress::MODE_CFG, modeConfiguration);
            result2 = other->write(RegisterAddress::MODE_CFG, modeConfiguration);

            if(result2 != Result::Ok)
            {
                result1 = result2;
            }
        }
        else
        {
            result1 = write(RegisterAddress::MODE_CFG, modeConfiguration);
        }

        return result1;
    }

    ENS220::Result ENS220::stopContinuousMeasure()
    {
        return reset();
    }

    void ENS220::enableDebugging(Stream& debugStream)
    {
        this->debugStream = &debugStream;
    }

    void ENS220::disableDebugging()
    {
        debugStream = nullptr;
    }

    template<class T>
    ENS220::Result ENS220::read(RegisterAddress address, T& data)
    {
        return read(address, (uint8_t*)&data, sizeof(data));
    }

    template<class T>
    ENS220::Result ENS220::write(RegisterAddress address, T data)
    {
        return write(address, (uint8_t*)&data, sizeof(data));
    }
}