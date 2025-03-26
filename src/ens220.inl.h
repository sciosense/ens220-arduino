#include "ens220.h"


#define hasFlag(a, b) (((a) & (b)) == (b))

inline ENS220::~ENS220() { }

inline ENS220::ENS220()
{
    debugStream         = nullptr;
    interruptPin        = NOT_A_PIN;

    partId              = 0;
    uid                 = 0;
    dataStatus          = ENS220_DATA_STATUS_NONE;
    standbyTime         = ENS220_STANDBY_TIME_ONE_SHOT_OPERATION;
    interruptStatus     = ENS220_INTERRUPT_STATUS_NONE;
    modeConfiguration   = (Ens220_ModeConfiguration)~ENS220_MODE_CONFIGURATION_MASK;

    setDefaultConfiguration();
}

bool ENS220::isConnected()
{
    return Ens220_IsConnected(this);
}

void ENS220::begin(TwoWire* wire, const uint8_t address)
{
    i2cConfig           = { 0 };
    i2cConfig.wire      = wire;
    i2cConfig.address   = address;

    io.read             = ScioSense_Arduino_I2c_Read;
    io.write            = ScioSense_Arduino_I2c_Write;
    io.wait             = ScioSense_Arduino_I2c_Wait;
    io.config           = &i2cConfig;
}

void ENS220::begin(SPIClass* spi, uint8_t chipSelect, const bool spi3WireMode, const SPISettings settings)
{
    spiConfig               = { 0 };
    spiConfig.spi           = spi;
    spiConfig.csPin         = chipSelect;
    spiConfig.settings      = settings;
    spiConfig.useSpiSettings= true;

    io.read             = ScioSense_Arduino_Spi_Read;
    io.write            = ScioSense_Arduino_Spi_Write;
    io.wait             = ScioSense_Arduino_Spi_Wait;
    io.spi3WireMode     = spi3WireMode;
    io.config           = &spiConfig;

    //disable i2c and enable spi by setting CSn line low;
    pinMode(spiConfig.csPin, OUTPUT);
    digitalWrite(spiConfig.csPin, LOW);
    delay(1);
    digitalWrite(spiConfig.csPin, HIGH);
}

bool ENS220::init()
{
    Result result;

    result = Ens220_Init(this);

    debug(__func__, result);
    return isConnected();
}

Result ENS220::reset()
{
    Result result;

    result = Ens220_Reset(this);

    debug(__func__, result);
    return result;
}

Result ENS220::setHighPower(bool enable)
{
    Result result;

    result = Ens220_SetHighPower(this, enable);

    debug(__func__, result);
    return result;
}

Result ENS220::writeConfiguration()
{
    Result result;

    result = Ens220_WriteConfiguration(this);

    debug(__func__, result);
    return result;
}

Result ENS220::updateInterrupted()
{
    Result result;

    result = Ens220_UpdateInterrupted(this);

    debug(__func__, result);
    return result;
}
Result ENS220::updatePolled()
{
    Result result;

    result = Ens220_UpdatePolled(this);

    debug(__func__, result);
    return result;
}

Result ENS220::singleShotMeasure(Ens220_Sensor sensor, ScioSense_Ens220* other)
{
    Result result;

    result = Ens220_SingleShotMeasure(this, sensor, other);

    debug(__func__, result);
    return result;
}

Result ENS220::startContinuousMeasure(Ens220_Sensor sensor, ScioSense_Ens220* other)
{
    Result result;

    result = Ens220_StartContinuousMeasure(this, sensor, other);

    debug(__func__, result);
    return result;
}

Result ENS220::stopContinuousMeasure()
{
    return Ens220_StopContinuousMeasure(this);
}

void ENS220::enableDebugging(Stream& debugStream)
{
    this->debugStream = &debugStream;
}

void ENS220::disableDebugging()
{
    debugStream = nullptr;
}

Result ENS220::update()
{
    Result result;

    result = Ens220_Update(this);

    debug(__func__, result);
    return result;
}

const uint8_t* ENS220::getRawConfiguration()
{
    return Ens220_GetRawConfiguration(this);
}

uint8_t ENS220::getFifoThreshold()
{
    return Ens220_GetFifoThreshold(this);
}

Ens220_PressureConversionTime ENS220::getPressureConversionTime()
{
    return Ens220_GetPressureConversionTime(this);
}

Ens220_Oversampling ENS220::getOversamplingOfPressure()
{
    return Ens220_GetOversamplingOfPressure(this);
}

Ens220_Oversampling ENS220::getOversamplingOfTemperature()
{
    return Ens220_GetOversamplingOfTemperature(this);
}

uint32_t ENS220::getUID()
{
    return Ens220_GetUid(this);
}

void ENS220::setDefaultConfiguration()
{
    Ens220_SetDefaultConfiguration(this);
}

void ENS220::setRawConfiguration(const uint8_t* config)
{
    Ens220_SetRawConfiguration(this, config);
};

void ENS220::setPressureDataPath(const Ens220_PressureDataPath& dataPath)
{
    Ens220_SetPressureDataPath(this, dataPath);
}

void ENS220::setPressureConversionTime(const Ens220_PressureConversionTime& time)
{
    Ens220_SetPressureConversionTime(this, time);
};

void ENS220::setPressureTemperatureRatio(const Ens220_PressureTemperatureRatio& ratio)
{
    Ens220_SetPressureTemperatureRatio(this, ratio);
};

void ENS220::setStandbyTime(const Ens220_StandbyTime& time)
{
    Ens220_SetStandbyTime(this, time);
};

void ENS220::setOversamplingOfPressure(const Ens220_Oversampling& n)
{
    Ens220_SetOversamplingOfPressure(this, n);
};

void ENS220::setOversamplingOfTemperature (const Ens220_Oversampling& n)
{
    Ens220_SetOversamplingOfTemperature(this, n);
};

void ENS220::setMovingAverageSamples(const Ens220_MovingAverageSamples& n)
{
    Ens220_SetMovingAverageSamples(this, n);
};

void ENS220::setInterfaceConfiguration(const Ens220_InterfaceConfiguration& interfaceConf)
{
    Ens220_SetInterfaceConfiguration(this, interfaceConf);
};

void ENS220::setInterruptConfiguration(const Ens220_InterruptConfiguration& interruptConf)
{
    Ens220_SetInterruptConfiguration(this, interruptConf);
};

void ENS220::setLowPressThreshold(const uint32_t& threshold)
{
    Ens220_SetLowPressThreshold(this, threshold);
};
void ENS220::setHighPressThreshold(const uint32_t& threshold)
{
    Ens220_SetHighPressThreshold(this, threshold);
};

void ENS220::setFifoThreshold(const uint8_t& threshold)
{
    Ens220_SetFifoThreshold(this, threshold);
};

void ENS220::setInterruptPin(int p)
{
    pinMode(p, INPUT);
    interruptPin = p;
}

inline uint16_t ENS220::getTempRaw() const
{
    return Ens220_GetTempRaw(this);
}

float ENS220::getTempKelvin()
{
    return Ens220_GetTempKelvin(this);
}

float ENS220::getTempCelsius()
{
    return Ens220_GetTempCelsius(this);
}

float ENS220::getTempFahrenheit()
{
    return Ens220_GetTempFahrenheit(this);
}

float ENS220::getPressurePascal()
{
    return Ens220_GetPressurePascal(this);
}

float ENS220::getPressurePascal(uint8_t index)
{
    return Ens220_GetPressurePascalFifo(this, index);
}

float ENS220::getPressureHectoPascal()
{
    return Ens220_GetPressureHectoPascal(this);
}

float ENS220::getPressureHectoPascal(uint8_t index)
{
    return Ens220_GetPressureHectoPascalFifo(this, index);
}

uint32_t ENS220::getPressureRaw()
{
    return Ens220_GetPressureRaw(this);
}

uint32_t ENS220::getPressureRaw(const uint8_t& fifoIndex) const
{
    return Ens220_GetPressureRawFifo(this, fifoIndex);
}

const uint8_t* ENS220::getPressureBuffer()
{
    return Ens220_GetPressureBuffer(this);
}

Ens220_DataStatus ENS220::getDataStatus()
{
    return Ens220_GetDataStatus(this);
}

bool ENS220::hasDataStatusFlag(Ens220_DataStatus flag)
{
    return ((Ens220_GetDataStatus(this) & flag) == flag);
}

Ens220_InterruptStatus ENS220::getInterruptStatus()
{
    return Ens220_GetInterruptStatus(this);
}

bool ENS220::hasInterruptStatusFlag(Ens220_InterruptStatus flag)
{
    return ((Ens220_GetInterruptStatus(this) & flag) == flag);
}

uint16_t ENS220::CalculateMeasurementTimeSingleShot(const Ens220_Sensor& sensor)
{
    return Ens220_CalculateMeasurementTimeSingleShot(this, sensor);
}

float ENS220::CalculateDryAirDensity(const uint8_t& fifoIndex) const
{
    return Ens220_CalculateDryAirDensity(this, fifoIndex);
}

float ENS220::CalculateDryAirDensity(const ScioSense_Ens220& other, const uint8_t& fifoIndex) const
{
    return Ens220_CalculateDryAirDensity2(this, &other, fifoIndex);
}

float ENS220::CalculateHeightDifference(const ScioSense_Ens220& other, const uint8_t& fifoIndex) const
{
    return Ens220_CalculateHeightDifference(this, &other, fifoIndex);
}

float ENS220::CalculateHeightDifference(const float& dryAirDensity, const ScioSense_Ens220& other, const uint8_t& fifoIndex) const
{
    return Ens220_CalculateHeightDifference2(this, dryAirDensity, &other, fifoIndex);
}

float ENS220::CalculateHeightDifference(const float& dryAirDensity, const uint32_t& p1Raw, const uint32_t& p2Raw) const
{
    return Ens220_CalculateHeightDifference3(dryAirDensity, p1Raw, p2Raw);
}

void ENS220::waitInterrupt()
{
    if (interruptPin != 0)
    {
        if (hasFlag((Ens220_InterfaceConfiguration)configurationBuffer[ENS220_BUFFER_INFO_INTF_CFG], ENS220_INTERFACE_CONFIGURATION_INTERRUPT_POLARITY))
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
    Ens220_WaitStandbyTime(this);
}

void ENS220::waitSingleShot()
{
    Ens220_WaitSingleShot(this);
}

Result ENS220::read(Ens220_RegisterAddress address, uint8_t* data, size_t size)
{
    return Ens220_Read(this, address, data, size);
}

Result ENS220::write(Ens220_RegisterAddress address, uint8_t* data, size_t size)
{
    return Ens220_Write(this, address, data, size);
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

const char* ENS220::toString(const Result& result)
{
    switch (result)
    {
        case RESULT_NOT_ALLOWED     : return "not-allowed";
        case RESULT_IO_ERROR        : return "io-error";
        case RESULT_CHECKSUM_ERROR  : return "checksum-error";
        case RESULT_INVALID        : return "data-invalid";
        case RESULT_OK             : return "ok";
        default                     : return "unknown-status";
    }
}

#undef hasFlag