#ifndef __SPARKFUN_BMP384_H__
#define __SPARKFUN_BMP384_H__

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

#include "bmp3_api/bmp3.h"

// SparkFun's default I2C address is opposite of Bosch's default
#define BMP384_I2C_ADDRESS_DEFAULT BMP3_ADDR_I2C_SEC    // 0x77
#define BMP384_I2C_ADDRESS_SECONDARY BMP3_ADDR_I2C_PRIM // 0x76

// Struct to hold
struct BMP384_InterfaceData
{
    // Communication interface (I2C or SPI)
    bmp3_intf interface;

    // I2C settings
    uint8_t i2cAddress;

    // SPI settings
    uint8_t spiCSPin;
    uint32_t spiClockFrequency;
};

class BMP384
{
    public:
        // Constructor
        BMP384();

        // Sensor initialization, must specify communication interface
        int8_t beginI2C(uint8_t address = BMP384_I2C_ADDRESS_DEFAULT);
        int8_t beginSPI(uint8_t csPin, uint32_t clockFrequency = 100000);

        // Data acquisistion
        bmp3_data getSensorData();

    private:
        // Sensor initialization, after communication interface has been selected
        int8_t begin();

        // Read/write helper functions
        static BMP3_INTF_RET_TYPE readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);
        static BMP3_INTF_RET_TYPE writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr);

        // Deley helper function
        static void usDelay(uint32_t period, void* interfacePtr);

        // Reference to the sensor
        struct bmp3_dev sensor;

        // Information about the selected communication interface (I2C or SPI)
        BMP384_InterfaceData interfaceData;
};

#endif