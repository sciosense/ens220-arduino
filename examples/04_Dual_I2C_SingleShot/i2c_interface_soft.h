#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include "SlowSoftI2CMaster.h"
#include <ens220.h>

using namespace ScioSense;

class I2cInterface : public virtual Utils::IoInterface<ENS220::RegisterAddress, ENS220::Result>
{
public:
    I2cInterface();
public:
    void begin(uint8_t address = 0x20, uint8_t SDA_PIN = 0, uint8_t SCL_PIN = 0);
public:
    virtual ENS220::Result read(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size);
    virtual ENS220::Result write(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size);
private:
    uint8_t slaveAddress;
    SlowSoftI2CMaster *softI2C;
};

I2cInterface::I2cInterface()
{
    softI2C             = nullptr;
    slaveAddress        = 0x20;
}

void I2cInterface::begin(uint8_t address, uint8_t SDA_PIN, uint8_t SCL_PIN)
{
    softI2C = new SlowSoftI2CMaster(SDA_PIN, SCL_PIN, true);    // Set pullups
    if(!softI2C->i2c_init())
    {
        Serial.println("Software I2C init failed");
    }

    slaveAddress = address;
}

ENS220::Result I2cInterface::read(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    ENS220::Result result = ENS220::Result::IOError;

    if (size == 0)
    {
        return result;
    }
    if( softI2C->i2c_start((slaveAddress<<1)|I2C_WRITE) )
    {
        // the FIFO buffer of ENS220 is 3*32 bytes in size and should be read in multiples of 3.
        // the SlowSoftI2CMaster library can only read one byte at the time
        
        size_t len = 0;
        softI2C->i2c_write((uint8_t)address); // send desired register to device        
        while (len < size)
        {
            softI2C->i2c_rep_start((slaveAddress<<1)|I2C_READ); // restart for reading
            data[len] = softI2C->i2c_read(true); // read one byte and send NAK afterwards
            len++;
        }
        
        softI2C->i2c_stop(); // stop communication

        if (len == size)
        {
            result = ENS220::Result::Ok;
        }

    }

    return result;
}

ENS220::Result I2cInterface::write(const ENS220::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    ENS220::Result result = ENS220::Result::IOError;

    if( softI2C->i2c_start((slaveAddress<<1)|I2C_WRITE) )
    {
        int size_int = (int)size;
        softI2C->i2c_write((uint8_t)address);    // send desired register to device   
        for( int i=0; i<size_int; i++)
        {
            softI2C->i2c_write(data[i]);         // send data
        }
        
        softI2C->i2c_stop(); // stop communication
        
        result = ENS220::Result::Ok;
    }

    return result;
}

#endif //I2C_INTERFACE_H