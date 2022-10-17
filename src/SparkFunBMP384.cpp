#include "SparkFunBMP384.h"

BMP384::BMP384()
{
    // Nothing to do
}

int8_t BMP384::begin()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Initialize the sensor
    err = init();
    if(err != BMP3_OK)
    {
        return err;
    }

    // Enable both pressure and temperature sensors
    err = enablePressAndTemp(BMP3_ENABLE, BMP3_ENABLE);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Set to normal mode
    return setMode(BMP3_MODE_NORMAL);
}

int8_t BMP384::beginI2C(uint8_t address, TwoWire& wirePort)
{
    // Check whether address is valid option
    if(address != BMP3_ADDR_I2C_PRIM && address != BMP3_ADDR_I2C_SEC)
    {
        // Invalid option, don't do anything
        return BMP3_E_CONFIGURATION_ERR;
    }

    // Address is valid option
    interfaceData.i2cAddress = address;
    interfaceData.i2cPort = &wirePort;

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

int8_t BMP384::init()
{
    // Set helper function pointers
    sensor.read = readRegisters;
    sensor.write = writeRegisters;
    sensor.delay_us = usDelay;
    sensor.intf_ptr = &interfaceData;

    // Initialize the sensor
    return bmp3_init(&sensor);
}

int8_t BMP384::setMode(uint8_t mode)
{
    // Check whether this is a valid mode
    if(mode != BMP3_MODE_SLEEP && mode != BMP3_MODE_FORCED && mode != BMP3_MODE_NORMAL)
    {
        // Invalid mode
        return BMP3_E_CONFIGURATION_ERR;
    }

    // Mode is valid, set sensor mode as requested
    struct bmp3_settings settings;
    settings.op_mode = mode;
    return bmp3_set_op_mode(&settings, &sensor);
}

int8_t BMP384::enablePressAndTemp(uint8_t pressEnable, uint8_t tempEnable)
{
    // Set up default settings
    struct bmp3_settings settings;
    settings.press_en = pressEnable;
    settings.temp_en = tempEnable;

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN;

    // Set sensor settings
    return bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
}

int8_t BMP384::getSensorData(bmp3_data* data)
{
    return bmp3_get_sensor_data(BMP3_PRESS_TEMP, data, &sensor);
}

int8_t BMP384::getSensorStatus(bmp3_sens_status* sensorStatus)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Grab status values from the sensor
    bmp3_status status;
    err = bmp3_get_status(&status, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Copy over the sensor status only
    memcpy(sensorStatus, &status.sensor, sizeof(bmp3_sens_status));

    return BMP3_OK;
}

int8_t BMP384::setODRFrequency(uint8_t odr)
{
    // Check whether ODR is valid
    if(odr > BMP3_ODR_0_001_HZ)
    {
        return BMP3_E_INVALID_ODR_OSR_SETTINGS;
    }

    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Grab current OSR settings
    struct bmp3_odr_filter_settings osrMultipliers;
    err = getOSRMultipliers(&osrMultipliers);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Check whether this ODR is allowable for current OSR setting
    if(odr < calculateMinODR(osrMultipliers))
    {
        return BMP3_E_INVALID_ODR_OSR_SETTINGS;
    }
    
    // Set up ODR settings
    struct bmp3_settings settings;
    settings.odr_filter.odr = odr;

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_ODR;

    // Set sensor settings
    return bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
}

int8_t BMP384::getODRFrequency(uint8_t* odr)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Get sensor settings
    struct bmp3_settings settings;
    err =  bmp3_get_sensor_settings(&settings, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Store ODR setting
    *odr = settings.odr_filter.odr;

    return BMP3_OK;
}

int8_t BMP384::setOSRMultipliers(bmp3_odr_filter_settings osrMultipliers)
{
    // Check whether OSR multipliers are valid
    if(osrMultipliers.press_os > BMP3_OVERSAMPLING_32X
        || osrMultipliers.temp_os > BMP3_OVERSAMPLING_32X)
    {
        return BMP3_E_INVALID_ODR_OSR_SETTINGS;
    }
    
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;
    
    // Set up OSR settings
    struct bmp3_settings settings;
    settings.odr_filter.press_os = osrMultipliers.press_os;
    settings.odr_filter.temp_os = osrMultipliers.temp_os;

    // Grab current ODR, it may need to be increased for these OSR settings
    err = getODRFrequency(&settings.odr_filter.odr);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Determine the minimum required ODR for these OSR multipliers
    uint8_t minODR = calculateMinODR(osrMultipliers);

    // Update ODR if needed
    if(settings.odr_filter.odr < minODR)
    {
        settings.odr_filter.odr = minODR;
    }

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_TEMP_OS | BMP3_SEL_PRESS_OS | BMP3_SEL_ODR;

    // Set sensor settings
    return bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
}

int8_t BMP384::getOSRMultipliers(bmp3_odr_filter_settings* osrMultipliers)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Get sensor settings
    struct bmp3_settings settings;
    err =  bmp3_get_sensor_settings(&settings, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Store OSR multipliers
    osrMultipliers->press_os = settings.odr_filter.press_os;
    osrMultipliers->temp_os = settings.odr_filter.temp_os;

    return BMP3_OK;
}

int8_t BMP384::setFilterCoefficient(uint8_t coefficient)
{
    // Check whether coefficient is valid
    if(coefficient > BMP3_IIR_FILTER_COEFF_127)
    {
        return BMP3_E_INVALID_ODR_OSR_SETTINGS;
    }
    
    // Set up filter settings
    struct bmp3_settings settings;
    settings.odr_filter.iir_filter = coefficient;

    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_IIR_FILTER;

    // Set sensor settings
    return bmp3_set_sensor_settings(settingsMask, &settings, &sensor);
}

int8_t BMP384::setInterruptSettings(bmp3_int_ctrl_settings interruptSettings)
{
    // Set up interrupt settings
    struct bmp3_settings settings;
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
    bmp3_status status;
    err = bmp3_get_status(&status, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Copy over the interrupt status only
    memcpy(interruptStatus, &status.intr, sizeof(bmp3_int_status));

    return BMP3_OK;
}

int8_t BMP384::setFIFOSettings(bmp3_fifo_settings fifoSettings)
{
    // Create bit mask for which settings we want to change
    uint16_t settingsMask = BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_STOP_ON_FULL_EN
        | BMP3_SEL_FIFO_TIME_EN | BMP3_SEL_FIFO_PRESS_EN | BMP3_SEL_FIFO_TEMP_EN
        | BMP3_SEL_FIFO_DOWN_SAMPLING | BMP3_SEL_FIFO_FILTER_EN
        | BMP3_SEL_FIFO_FWTM_EN | BMP3_SEL_FIFO_FULL_EN ;

    // Set sensor settings
    return bmp3_set_fifo_settings(settingsMask, &fifoSettings, &sensor);
}

int8_t BMP384::setFIFOWatermark(uint8_t numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Set requested number of data frames
    bmp3_fifo_data fifoData;
    fifoData.req_frames = numData;

    // Get current FIFO settings
    bmp3_fifo_settings fifoSettings;
    bmp3_get_fifo_settings(&fifoSettings, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Set watermark as requested
    return bmp3_set_fifo_watermark(&fifoData, &fifoSettings, &sensor);
}

int8_t BMP384::getFIFOLength(uint8_t* numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Grab FIFO length in bytes
    uint16_t fifoLengthBytes = 0;
    err = bmp3_get_fifo_length(&fifoLengthBytes, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Get current FIFO settings
    bmp3_fifo_settings fifoSettings;
    bmp3_get_fifo_settings(&fifoSettings, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Determine the number of bytes per data frame.
    // 1 byte for header, 3 bytes for each sensor data
    uint16_t bytesPerFrame = 1 + 3 * (fifoSettings.press_en + fifoSettings.temp_en);

    // Set number of data frames
    *numData = fifoLengthBytes / bytesPerFrame;

    return BMP3_OK;
}

int8_t BMP384::getFIFOData(bmp3_data* data, uint8_t numData)
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Get current FIFO settings
    bmp3_fifo_settings fifoSettings;
    bmp3_get_fifo_settings(&fifoSettings, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Create buffer to hold all bytes of FIFO buffer. bmp3_get_fifo_data() calls
    // reset_fifo_index(), which will zero out 512 bytes of this buffer, so this
    // needs to be 512 bytes!
    uint8_t byteBuffer[512];

    // Get all bytes out of FIFO buffer
    bmp3_fifo_data fifoData;
    fifoData.buffer = byteBuffer;
    err = bmp3_get_fifo_data(&fifoData, &fifoSettings, &sensor);
    if(err != BMP3_OK)
    {
        return err;
    }

    // Determine the number of bytes per data frame.
    // 1 byte for header, 3 bytes for each sensor data
    uint16_t bytesPerFrame = 1 + 3 * (fifoSettings.press_en + fifoSettings.temp_en);

    // bmp3_extract_fifo_data() will parse all bytes in byteBuffer into data frames.
    // The buffer supplied to this function may not be big enough for all those
    // frames, so we create our own frame buffer to definitely take it all
    bmp3_data frameBuffer[fifoData.byte_count / bytesPerFrame];

    // Parse raw data into temperature and pressure data
    err = bmp3_extract_fifo_data(frameBuffer, &fifoData, &sensor);
    if(err < BMP3_OK)
    {
        return err;
    }

    // Now copy all requested data into the caller's buffer
    for(uint8_t i = 0; i < numData; i++)
    {
        data[i] = frameBuffer[i];
    }

    return err;
}

int8_t BMP384::flushFIFO()
{
    return bmp3_fifo_flush(&sensor);
}

uint8_t BMP384::calculateMinODR(bmp3_odr_filter_settings osrMultipliers)
{
    // Compute the measurement time in microseconds, as per the datasheet's equation
    float measureTime = 234;
    measureTime += BMP3_SETTLE_TIME_PRESS + pow(2, osrMultipliers.press_os) * BMP3_ADC_CONV_TIME;
    measureTime += BMP3_SETTLE_TIME_TEMP + pow(2, osrMultipliers.temp_os) * BMP3_ADC_CONV_TIME;

    // Max possible readout frequency
    float maxFreq = pow(10, 6) / measureTime;

    // Minimum possible ODR setting
    float minODR = ceil(log2(200 / maxFreq));

    return minODR;
}

float BMP384::log2(float x)
{
    // Change of base rule of logarithms
    return log(x) / log(2);
}

BMP3_INTF_RET_TYPE BMP384::readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr)
{
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return BMP3_E_INVALID_LEN;
    }

    // Get interface data
    BMP384_InterfaceData* interfaceData = (BMP384_InterfaceData*) interfacePtr;

    switch(interfaceData->interface)
    {
        case BMP3_I2C_INTF:
            // Jump to desired register address
            interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);
            interfaceData->i2cPort->write(regAddress);
            if(interfaceData->i2cPort->endTransmission())
            {
                return BMP3_E_COMM_FAIL;
            }

            // Read bytes from these registers
            interfaceData->i2cPort->requestFrom(interfaceData->i2cAddress, numBytes);

            // Store all requested bytes
            for(uint32_t i = 0; i < numBytes && interfaceData->i2cPort->available(); i++)
            {
                dataBuffer[i] = interfaceData->i2cPort->read();
            }
            break;

        case BMP3_SPI_INTF:
            // Start transmission
            SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
            digitalWrite(interfaceData->spiCSPin, LOW);
            SPI.transfer(regAddress | 0x80);

            // Read all requested bytes
            for(uint32_t i = 0; i < numBytes; i++)
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
    // Make sure the number of bytes is valid
    if(numBytes == 0)
    {
        return BMP3_E_INVALID_LEN;
    }
    // Get interface data
    BMP384_InterfaceData* interfaceData = (BMP384_InterfaceData*) interfacePtr;

    // Determine which interface we're using
    switch(interfaceData->interface)
    {
        case BMP3_I2C_INTF:
            // Begin transmission
            interfaceData->i2cPort->beginTransmission(interfaceData->i2cAddress);

            // Write the address
            interfaceData->i2cPort->write(regAddress);
            
            // Write all the data
            for(uint32_t i = 0; i < numBytes; i++)
            {
                interfaceData->i2cPort->write(dataBuffer[i]);
            }

            // End transmission
            if(interfaceData->i2cPort->endTransmission())
            {
                return BMP3_E_COMM_FAIL;
            }
            break;

        case BMP3_SPI_INTF:
            // Begin transmission
            SPI.beginTransaction(SPISettings(interfaceData->spiClockFrequency, MSBFIRST, SPI_MODE0));
            digitalWrite(interfaceData->spiCSPin, LOW);
            
            // Write the address
            SPI.transfer(regAddress);
            
            // Write all the data
            for(uint32_t i = 0; i < numBytes; i++)
            {
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