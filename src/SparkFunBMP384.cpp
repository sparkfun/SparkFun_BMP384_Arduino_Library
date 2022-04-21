#include "SparkFunBMP384.h"

BMP384::BMP384()
{
    // Nothing to do
}

int8_t BMP384::begin()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

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

int8_t BMP384::getSensorData(bmp3_data* data)
{
    return bmp3_get_sensor_data(BMP3_PRESS_TEMP, data, &sensor);
}

int8_t BMP384::setODRFrequency(uint8_t odr)
{
    // Check whether ODR is valid
    if(odr > BMP3_ODR_0_001_HZ)
    {
        return BMP3_E_INVALID_ODR_OSR_SETTINGS;
    }
    
    // Set up ODR settings
    struct bmp3_settings settings = {0};
    settings.odr_filter.odr = odr;

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_ODR;

    // Set sensor settings
    return bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
}

int8_t BMP384::setInterruptSettings(bmp3_int_ctrl_settings interruptSettings)
{
    // Set up interrupt settings
    struct bmp3_settings settings = {0};
    settings.int_settings = interruptSettings;

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_OUTPUT_MODE | BMP3_SEL_LEVEL | BMP3_SEL_DRDY_EN;

    // Set sensor settings
    return bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
}

int8_t BMP384::getInterruptStatus(bmp3_int_status* interruptStatus)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Grab status values from the sensor
    bmp3_status status = {0};
    err = bmp3_get_status(&status, &sensor);
    if(err)
    {
        return err;
    }

    // Copy over the interrupt status only
    memcpy(interruptStatus, &status.intr, sizeof(bmp3_int_status));

    return BMP3_OK;
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