#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include "SlowSoftI2CMaster.h"
#include <ens220.h>

class ScioSense_Arduino_SlowSoftI2CMaster_Config
{
public:
    ScioSense_Arduino_SlowSoftI2CMaster_Config()
    {
        softI2C = nullptr;
    }
    ~ScioSense_Arduino_SlowSoftI2CMaster_Config()
    {
        if(softI2C)
        {
            delete softI2C;
        }
    }
    void init(uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t address)
    {
        if(softI2C)
        {
            delete softI2C;
        }

        softI2C = new SlowSoftI2CMaster(SDA_PIN, SCL_PIN, true);    // Set pullups
        if(!softI2C->i2c_init())
        {
            Serial.println("Software I2C init failed");
        }

        this->address = address;
    }

    SlowSoftI2CMaster* softI2C;
    unsigned char address;
};

inline int8_t ScioSense_Arduino_SlowSoftI2CMaster_Read(void* config, const uint16_t address, uint8_t* data, const size_t size)
{
    SlowSoftI2CMaster* softI2C  = ((ScioSense_Arduino_SlowSoftI2CMaster_Config*)config)->softI2C;
    unsigned char slaveAddress  = ((ScioSense_Arduino_SlowSoftI2CMaster_Config*)config)->address;

    Result result = RESULT_IO_ERROR;

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
            result = RESULT_OK;
        }
    }

    return result;
}

inline int8_t ScioSense_Arduino_SlowSoftI2CMaster_Write(void* config, const uint16_t address, uint8_t* data, const size_t size)
{
    SlowSoftI2CMaster* softI2C  = ((ScioSense_Arduino_SlowSoftI2CMaster_Config*)config)->softI2C;
    unsigned char slaveAddress  = ((ScioSense_Arduino_SlowSoftI2CMaster_Config*)config)->address;

    Result result = RESULT_IO_ERROR;

    if( softI2C->i2c_start((slaveAddress<<1)|I2C_WRITE) )
    {
        softI2C->i2c_write((uint8_t)address);    // send desired register to device
        for( size_t i=0; i<size; i++)
        {
            softI2C->i2c_write(data[i]);         // send data
        }

        softI2C->i2c_stop(); // stop communication

        result = RESULT_OK;
    }

    return result;
}

inline void ScioSense_Arduino_SlowSoftI2CMaster_Wait(uint32_t ms)
{
    delay(ms);
}

inline void ScioSense_Arduino_SlowSoftI2CMaster_begin(ENS220& ens220, ScioSense_Arduino_SlowSoftI2CMaster_Config& config, uint8_t address, uint8_t SDA_PIN, uint8_t SCL_PIN)
{
    config.init(SDA_PIN, SCL_PIN, address);

    ens220.io.read      = ScioSense_Arduino_SlowSoftI2CMaster_Read;
    ens220.io.write     = ScioSense_Arduino_SlowSoftI2CMaster_Write;
    ens220.io.wait      = ScioSense_Arduino_SlowSoftI2CMaster_Wait;
    ens220.io.config    = &config;
}

#endif // I2C_INTERFACE_H