#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <SPI.h>
#include <ens220.h>

using namespace ScioSense;

class SpiInterface : public virtual Utils::IoInterface<ENS220::RegisterAddress, ENS220::Result>
{
public:
    void begin(uint8_t chipSelect, uint32_t clock, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0);
public:
    virtual ENS220::Result read(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size);
    virtual ENS220::Result write(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size);
private:
    uint8_t csPin = 0;
};

void SpiInterface::begin(uint8_t chipSelect, uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
{
    csPin = chipSelect;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, LOW);
    delay(1);
    digitalWrite(csPin, HIGH);

    SPI.begin();
    SPI.beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
}

ENS220::Result SpiInterface::read(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    digitalWrite(csPin, LOW);
    {
        SPI.transfer(((uint8_t)address << 2) + 1);
        for (size_t i = 0; i < size; i++)
        {
            data[i] = SPI.transfer(0xff);
        }
    }
    digitalWrite(csPin, HIGH);

    return ENS220::Result::Ok;
}

ENS220::Result SpiInterface::write(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    digitalWrite(csPin, LOW);
    {
        SPI.transfer((uint8_t)address << 2);
        for (size_t i = 0; i < size; i++)
        {
            SPI.transfer(data[i]);
        }
    }
    digitalWrite(csPin, HIGH);

    return ENS220::Result::Ok;
}

#endif //SPI_INTERFACE_H