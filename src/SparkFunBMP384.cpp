#include "SparkFunBMP384.h"

BMP384::BMP384()
{
    // Nothing to do
}

int8_t BMP384::begin()
{
    // Variable to track errors returned by API calls
    int8_t err = 0;

    // Set helper function pointers
    sensor.read = readRegisters;
    sensor.write = writeRegisters;
    sensor.delay_us = usDelay;
    sensor.intf_ptr = &interfaceData;

    // Initialize the sensor
    err = bmp3_init(&sensor);
    if(err)
    {
        return err;
    }

    // Set up default settings
    struct bmp3_settings settings = {0};
    settings.temp_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN;

    // Set sensor settings
    err = bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
    if(err)
    {
        return err;
    }

    // Set to normal mode
    settings.op_mode = BMP3_MODE_NORMAL;
    err = bmp3_set_op_mode(&settings, &sensor);
    if(err)
    {
        return err;
    }

    return BMP3_OK;
}

int8_t BMP384::beginI2C(uint8_t address)
{
    // Check whether address is valid option
    if(address != BMP3_ADDR_I2C_PRIM && address != BMP3_ADDR_I2C_SEC)
    {
        // Invalid option, don't do anything
        return BMP3_E_CONFIGURATION_ERR;
    }

    // Address is valid option
    interfaceData.i2cAddress = address;

    // Set interface
    sensor.intf = BMP3_I2C_INTF;
    interfaceData.interface = BMP3_I2C_INTF;

    // Initialize sensor
    return begin();
}

int8_t BMP384::beginSPI(uint8_t csPin, uint32_t clockFrequency)
{
    // Set up chip select pin
    interfaceData.spiCSPin = csPin;
    digitalWrite(csPin, HIGH); // Write high now to ensure pin doesn't go low
    pinMode(csPin, OUTPUT);

    // Set desired clock frequency
    interfaceData.spiClockFrequency = clockFrequency;

    // Set interface
    sensor.intf = BMP3_SPI_INTF;
    interfaceData.interface = BMP3_SPI_INTF;

    // Initialize sensor
    return begin();
}

bmp3_data BMP384::getSensorData()
{
    bmp3_data data = {0};
    bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &sensor);
    return data;
}

BMP3_INTF_RET_TYPE BMP384::readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    BMP384_InterfaceData* interfaceData = (BMP384_InterfaceData*) interfacePtr;

    switch(interfaceData->interface)
    {
        case BMP3_I2C_INTF:
            // Jump to desired register address
            Wire.beginTransmission(interfaceData->i2cAddress);
            Wire.write(regAddress);
            if(Wire.endTransmission())
            {
                return BMP3_E_COMM_FAIL;
            }

            // Read bytes from these registers
            Wire.requestFrom(interfaceData->i2cAddress, numBytes);

            // Store all requested bytes
            for(uint8_t i = 0; i < numBytes && Wire.available(); i++)
            {
                dataBuffer[i] = Wire.read();
            }
            break;

        case BMP3_SPI_INTF:
            // Start transmission
            SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
            digitalWrite(interfaceData->spiCSPin, LOW);
            SPI.transfer(regAddress | 0x80);

            // Read all requested bytes
            for(uint8_t i = 0; i < numBytes; i++)
            {
                dataBuffer[i] = SPI.transfer(0);;
            }

            // End transmission
            digitalWrite(interfaceData->spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }

    return BMP3_OK;
}

BMP3_INTF_RET_TYPE BMP384::writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    BMP384_InterfaceData* interfaceData = (BMP384_InterfaceData*) interfacePtr;

    switch(interfaceData->interface)
    {
        case BMP3_I2C_INTF:
            Wire.beginTransmission(interfaceData->i2cAddress);
            
            // Registers have to be written in address/data pairs
            for(uint8_t i = 0; i < numBytes; i++)
            {
                Wire.write(regAddress + i);
                Wire.write(dataBuffer[i]);
            }

            if(Wire.endTransmission())
            {
                return BMP3_E_COMM_FAIL;
            }
            break;

        case BMP3_SPI_INTF:
            // Start transmission
            SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
            digitalWrite(interfaceData->spiCSPin, LOW);
            
            // Registers have to be written in address/data pairs
            for(uint8_t i = 0; i < numBytes; i++)
            {
                SPI.transfer(regAddress + i);
                SPI.transfer(dataBuffer[i]);
            }

            // End transmission
            digitalWrite(interfaceData->spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }

    return BMP3_OK;
}

void BMP384::usDelay(uint32_t period, void* interfacePtr)
{
    delayMicroseconds(period);
}