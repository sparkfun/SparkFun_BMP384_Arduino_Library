#include "SparkFunBMP384.h"

BMP384::BMP384()
{
    // Nothing to do
}

bool BMP384::begin()
{
    // BMP384 requires 2ms start-up time
    delay(2);

    // Check whether chip ID is correct
    if(getChipID() != BMP384_CHIP_ID)
    {
        // Chip ID is wrong, we're not connected to the BMP384
        return false;
    }

    // Trigger a reset, just in case some previous configuration is active
    reset();

    // We're connected to the chip, we need to grab the calibration data
    getCalibrationData();

    // Set to active mode
    setPowerMode(BMP384_POWER_MODE_NORMAL);
    
    // Enable temperature and pressure sensors
    enableTemperature(true);
    enablePressure(true);

    return true;
}

bool BMP384::beginI2C(uint8_t address)
{
    communicationInterface = BMP348_COM_I2C;
    setI2CAddress(address);
    return begin();
}

bool BMP384::beginSPI(uint8_t csPin)
{
    communicationInterface = BMP348_COM_SPI;
    spiCSPin = csPin;
    digitalWrite(spiCSPin, HIGH);
    pinMode(spiCSPin, OUTPUT);
    return begin();
}

void BMP384::reset()
{
    writeRegister(BMP384_REG_CMD, BMP384_CMD_RESET);
}

void BMP384::setPowerMode(uint8_t modeBits)
{
    // Ensure only last 2 bits are used
    modeBits &= 0b11;

    // Zero out the mode bits
    regPwrCtrl &= ~(0b11 << 4);

    // Set mode bits as desired
    regPwrCtrl |= modeBits << 4;

    // Update register on device
    writeRegister(BMP384_REG_PWR_CTRL, regPwrCtrl);
}

void BMP384::enableTemperature(bool enable)
{
    // Zero out the temperature enable bit
    regPwrCtrl &= ~(0b1 << 1);

    // Set temperature enable bit as desired
    regPwrCtrl |= enable << 1;

    // Update register on device
    writeRegister(BMP384_REG_PWR_CTRL, regPwrCtrl);
}

void BMP384::enablePressure(bool enable)
{
    // Zero out the pressure enable bit
    regPwrCtrl &= ~(0b1 << 0);

    // Set pressure enable bit as desired
    regPwrCtrl |= enable << 0;

    // Update register on device
    writeRegister(BMP384_REG_PWR_CTRL, regPwrCtrl);
}

float BMP384::getTemperature()
{
    // Get raw data from sensor
    uint64_t rawData = getRawData();
    uint32_t rawTemp = (rawData >> 24) & 0xFFFFFF;

    // Convert to temperature in C
    return convertTemperature(rawTemp);
}

float BMP384::getPressure()
{
    // Get raw data from sensor
    uint64_t rawData = getRawData();
    uint32_t rawTemp = (rawData >> 24) & 0xFFFFFF;
    uint32_t rawPress = rawData & 0xFFFFFF;

    // Convert to temperature in C
    float trueTemp = convertTemperature(rawTemp);
    return convertPressure(rawPress, trueTemp);
}

uint8_t BMP384::getChipID()
{
    return readRegister(BMP384_REG_CHIP_ID);
}

bool BMP384::setI2CAddress(uint8_t address)
{
    // Check whether address is valid option
    if(address != BMP384_I2C_ADDRESS_0 && address != BMP384_I2C_ADDRESS_1)
    {
        // Invalid option, don't do anything
        return false;
    }

    // Address is valid option, set and return
    i2cAddress = address;
    return true;
}

BMP384_InterruptConfig BMP384::getInterruptConfig()
{
    BMP384_InterruptConfig config =
    {
        .registerVal = readRegister(BMP384_REG_INT_CTRL)
    };

    return config;
}

void BMP384::setInterruptConfig(BMP384_InterruptConfig config)
{
    writeRegister(BMP384_REG_INT_CTRL, config.registerVal);
}

BMP384_InterruptStatus BMP384::getInterruptStatus()
{
    BMP384_InterruptStatus status =
    {
        .registerVal = readRegister(BMP384_REG_INT_STATUS)
    };

    return status;
}

uint32_t BMP384::getODRPrescaler()
{
    uint8_t odr = readRegister(BMP384_REG_ODR);
    uint32_t prescaler = (uint32_t) 1 << odr;
    return prescaler;
}

void BMP384::setODRPrescaler(uint32_t prescaler)
{
    // Prescaler must be a power of 2, but user may give some other value.
    // If so, we'll truncate it by finding the previous greatest power of 2
    uint8_t odr = 0;
    while((prescaler >> odr) > 1)
    {
        // Next power of 2 is still less than prescaler
        odr++;

        // Don't go above max ODR value
        if(odr == 0x11)
        {
            break;
        }
    }

    writeRegister(BMP384_REG_ODR, odr);
}

uint8_t BMP384::getFilterCoefficient()
{
    // Get IIR bits
    uint8_t iir = (readRegister(BMP384_REG_CONFIG) >> 1) & 0b111;

    // Coefficient is 2^IIR - 1
    uint8_t coefficient = (1 << iir) - 1;
    return coefficient;
}

void BMP384::setFilterCoefficient(uint8_t coefficient)
{
    // Coefficient must be a power of 2 minus 1, but user may give some other value.
    // If so, we'll truncate it by finding the previous greatest power of 2 minus 1
    uint8_t iir = 0;
    while(((coefficient + 1) >> iir) > 1)
    {
        // Next power of 2 minus 1 is still less than coefficient
        iir++;

        // Don't go above max IIR value
        if(iir == 0b111)
        {
            break;
        }
    }
    
    writeRegister(BMP384_REG_CONFIG, (iir << 1));
}

uint8_t BMP384::getTemperatureOSRMultiplier()
{
    // Get temperature OSR bits
    uint8_t osr = (regOsr >> 3) & 0b111;

    // Multiplier is 2^OSR
    uint8_t multiplier = 1 << osr;
    return multiplier;
}

uint8_t BMP384::getPressureOSRMultiplier()
{
    // Get pressure OSR bits
    uint8_t osr = regOsr & 0b111;

    // Multiplier is 2^OSR
    uint8_t multiplier = 1 << osr;
    return multiplier;
}

void BMP384::setTemperatureOSRMultiplier(uint8_t multiplier)
{
    // Multipler must be a power of 2, but user may give some other value.
    // If so, we'll truncate it by finding the previous greatest power of 2
    uint8_t osr = 0;
    while((multiplier >> osr) > 1)
    {
        // Next power of 2 is still less than multiplier
        osr++;

        // Don't go above max OSR value
        if(osr == 0b101)
        {
            break;
        }
    }

    // Clear previous temperature OSR bits
    regOsr &= ~(0b111 << 3);
    
    // Set new OSR bits
    regOsr |= osr << 3;
    writeRegister(BMP384_REG_OSR, regOsr);
}

void BMP384::setPressureOSRMultiplier(uint8_t multiplier)
{
    // Multipler must be a power of 2, but user may give some other value.
    // If so, we'll truncate it by finding the previous greatest power of 2
    uint8_t osr = 0;
    while((multiplier >> osr) > 1)
    {
        // Next power of 2 is still less than multiplier
        osr++;

        // Don't go above max OSR value
        if(osr == 0b101)
        {
            break;
        }
    }

    // Clear previous temperature OSR bits
    regOsr &= ~0b111;
    
    // Set new OSR bits
    regOsr |= osr;
    writeRegister(BMP384_REG_OSR, regOsr);
}

void BMP384::setFIFOConfig(BMP384_FIFOConfig config)
{
    writeRegisters(BMP384_REG_FIFO_CONFIG_1, &config.registerVals, sizeof(config.registerVals));
    
    // Set the expected number of bytes per FIFO frame. Each frame has a 1 byte
    // header, plus 3 bytes per sensor
    numBytesPerFIFOFrame = 1 + 3 * (config.flags.tempEnable + config.flags.pressEnable);
}

void BMP384::setFIFOWatermarkBytes(uint16_t numBytes)
{
    // Watermark can't be greater than 9 bits long (511 max)
    if(numBytes > 0x1FF)
    {
        numBytes = 0x1FF;
    }
    writeRegisters(BMP384_REG_FIFO_WTM_0, &numBytes, sizeof(numBytes));
}

void BMP384::setFIFOWatermarkSamples(uint8_t numSamples)
{
    setFIFOWatermarkBytes(numSamples * numBytesPerFIFOFrame);
}

uint16_t BMP384::getFIFOLengthBytes()
{
    uint16_t numBytes = 0;
    readRegisters(BMP384_REG_FIFO_LENGTH_0, &numBytes, 2);
    return numBytes;
}

uint8_t BMP384::getFIFOLengthSamples()
{
    return getFIFOLengthBytes() / numBytesPerFIFOFrame;
}

int16_t BMP384::readFIFO(float* tempData, float* pressData, uint16_t numData)
{
    // Iterate through all requested data samples
    for(uint16_t i = 0; i < numData; i++)
    {
        // Read frame header. This is a partial read corner case described in the
        // datasheet, but we need to read the header separately to know how many
        // more bytes there are in this frame
        uint8_t header = readRegister(BMP384_REG_FIFO_DATA);

        // Determine type of frame by the first bit of the header
        if((header >> 7) & 1)
        {
            // This is a sensor frame, meaning we'll have some data bytes to read
            // Determine what data is in this frame
            bool s = (header >> 5) & 1;
            bool t = (header >> 4) & 1;
            bool p = (header >> 2) & 1;
            if(s)
            {
                // This is a sensor-time frame, which we don't support. These frames
                // only appear when the FIFO buffer is empty, so there's no more
                // data to be read
                return i;
            }
            else if(!(t | p))
            {
                // s=t=p=0, so this is an empty frame. These frames only appear
                // when the FIFO buffer is empty, so there's no more data to be read
                return i;
            }
            else
            {
                // This frame contains temperature and/or pressure data, each of
                // which consist of 3 bytes. Determine how many total bytes we
                // need to read out
                uint8_t numBytesToRead = 3 * (t + p);
                
                // Because we didn't read the entire frame, the header is repeated
                // during the next FIFO read
                numBytesToRead++;

                // Create a place to store the FIFO frame. Max length is 7 bytes
                uint8_t fifoFrame[7] = {0};

                // Read the frame. The address counter does not increment at the
                // FIFO data register
                readRegisters(BMP384_REG_FIFO_DATA, &fifoFrame, numBytesToRead);

                // Create a pointer to the start of the data within the frame. This
                // helps with copying into the output arrays, since if temperature
                // is disabled, those 3 bytes are not present
                uint8_t* dataPtr = fifoFrame + 1;

                // If the frame has temperature data, copy 3 bytes into the
                // temperature output array
                if(t)
                {
                    uint32_t rawTemp = 0;
                    memcpy(&rawTemp, dataPtr, 3);
                    tempData[i] = convertTemperature(rawTemp);
                    dataPtr += 3;
                }
                // If the frame has pressure data, copy 3 bytes into the
                // pressure output array
                if(p)
                {
                    uint32_t rawPress = 0;
                    memcpy(&rawPress, dataPtr, 3);
                    pressData[i] = convertPressure(rawPress, tempData[i]);
                }

                continue;
            }
        }
        else
        {
            // This is a control frame, determine what type
            uint8_t controlType = (header >> 2) & 0b11;
            if(controlType == 0b01)
            {
                // This is a configuration error frame, meaning the user has
                // not configured the FIFO buffer correctly. We need to remove
                // it from the buffer by reading it completely. These frames
                // are 2 bytes long
                int num = 2;
                uint8_t foo[2];
                readRegisters(BMP384_REG_FIFO_DATA, foo, 2);

                // Return an error code
                return -1;
            }
            else if(controlType == 0b10)
            {
                // This is a configuration change frame, indicating something
                // about the FIFO config has been modified between FIFO writes.
                // We won't do anything with this frame, but we need to remove it
                // from the buffer by reading it completely. The datasheet
                // doesn't specify how many bytes are in each config change
                // frame, but it appears to be 2 (1 header plus 1 data)
                int num = 2;
                uint8_t foo[2];
                readRegisters(BMP384_REG_FIFO_DATA, foo, 2);

                // Prevent index from incrementing, since this isn't a data frame
                i--;
                continue;
            }
            else
            {
                // Unknown frame, return an error code
                return -1;
            }
        }
    }

    // Data was successfully read out of the FIFO buffer
    return numData;
}

void BMP384::flushFIFO()
{
    writeRegister(BMP384_REG_CMD, BMP384_CMD_FIFO_FLUSH);
}

void BMP384::getCalibrationData()
{
    // Read raw calibration data stored on device
    BMP384_CalibrationRaw calibRaw;
    readRegisters(BMP384_REG_NVM_PAR_T1_LSB, &calibRaw, sizeof(calibRaw));

    // Convert raw data into floating point
    calibrationData.PAR_T1 = calibRaw.NVM_PAR_T1 / pow(2, -8);
    calibrationData.PAR_T2 = calibRaw.NVM_PAR_T2 / pow(2, 30);
    calibrationData.PAR_T3 = calibRaw.NVM_PAR_T3 / pow(2, 48);
    calibrationData.PAR_P1 = (calibRaw.NVM_PAR_P1 - pow(2, 14)) / pow(2, 20);
    calibrationData.PAR_P2 = (calibRaw.NVM_PAR_P2 - pow(2, 14)) / pow(2, 29);
    calibrationData.PAR_P3 = calibRaw.NVM_PAR_P3 / pow(2, 32);
    calibrationData.PAR_P4 = calibRaw.NVM_PAR_P4 / pow(2, 37);
    calibrationData.PAR_P5 = calibRaw.NVM_PAR_P5 / pow(2, -3);
    calibrationData.PAR_P6 = calibRaw.NVM_PAR_P6 / pow(2, 6);
    calibrationData.PAR_P7 = calibRaw.NVM_PAR_P7 / pow(2, 8);
    calibrationData.PAR_P8 = calibRaw.NVM_PAR_P8 / pow(2, 15);
    calibrationData.PAR_P9 = calibRaw.NVM_PAR_P9 / pow(2, 48);
    calibrationData.PAR_P10 = calibRaw.NVM_PAR_P10 / pow(2, 48);
    calibrationData.PAR_P11 = calibRaw.NVM_PAR_P11 / pow(2, 65);
}

uint64_t BMP384::getRawData()
{
    uint64_t rawData = 0;

    // Raw data is stored in 6 consecutive registers
    readRegisters(BMP384_REG_DATA0, &rawData, 6);

    return rawData;
}

float BMP384::convertTemperature(uint32_t rawTemp)
{
    // Temporary variables
    float partial1;
    float partial2;
    float trueTemp;

    // Compute true temperature, as per datasheet's equations
    partial1 = rawTemp - calibrationData.PAR_T1;
    partial2 = partial1 * calibrationData.PAR_T2;
    trueTemp = partial2 + pow(partial1, 2) * calibrationData.PAR_T3;

    // Return true temperature in C
    return trueTemp;
}

float BMP384::convertPressure(uint32_t rawPress, float trueTemp)
{
    // Temporary variables
    float partial1;
    float partial2;
    float partial3;
    float partial4;
    float partOut1;
    float partOut2;
    float truePress;

    // Compute true pressure, as per datasheet's equations
    partial1 = calibrationData.PAR_P6 * trueTemp;
    partial2 = calibrationData.PAR_P7 * pow(trueTemp, 2);
    partial3 = calibrationData.PAR_P8 * pow(trueTemp, 3);
    partOut1 = calibrationData.PAR_P5 + partial1 + partial2 + partial3;

    partial1 = calibrationData.PAR_P2 * trueTemp;
    partial2 = calibrationData.PAR_P3 * pow(trueTemp, 2);
    partial3 = calibrationData.PAR_P4 * pow(trueTemp, 3);
    partOut2 = rawPress * (calibrationData.PAR_P1 + partial1 + partial2 + partial3);

    partial1 = pow(rawPress, 2);
    partial2 = calibrationData.PAR_P9 + calibrationData.PAR_P10 * trueTemp;
    partial3 = partial1 * partial2;
    partial4 = partial3 + pow(rawPress, 3) * calibrationData.PAR_P11;
    truePress = partOut1 + partOut2 + partial4;

    // Return true pressure in Pa
    return truePress;
}

uint8_t BMP384::readRegister(uint8_t regAddress)
{
    // Read 1 byte at requested address
    uint8_t data = 0;
    readRegisters(regAddress, &data, 1);
    return data;
}

void BMP384::readRegisters(uint8_t regAddress, void* dataBuffer, uint16_t numBytes)
{
    switch(communicationInterface)
    {
        case BMP348_COM_I2C:
            // Jump to desired register address
            Wire.beginTransmission(i2cAddress);
            Wire.write(regAddress);
            Wire.endTransmission();

            // Read bytes from these registers
            Wire.requestFrom(i2cAddress, numBytes);

            // Store all requested bytes
            for(uint8_t i = 0; i < numBytes; i++)
            {
                ((uint8_t*) dataBuffer)[i] = Wire.read();
            }
            break;
        
        case BMP348_COM_SPI:
            SPI.beginTransaction(spiSettings);
            digitalWrite(spiCSPin, LOW);
            SPI.transfer(regAddress | 0x80);

            // Have to send one dummy byte
            SPI.transfer(0);

            // Store all requested bytes
            for(uint8_t i = 0; i < numBytes; i++)
            {
                ((uint8_t*) dataBuffer)[i] = SPI.transfer(0);;
            }

            digitalWrite(spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }
}

void BMP384::writeRegister(uint8_t regAddress, uint8_t data)
{
    // Write 1 byte at requested address
    writeRegisters(regAddress, &data, 1);
}

void BMP384::writeRegisters(uint8_t regAddress, void* data, uint8_t numBytes)
{
    switch(communicationInterface)
    {
        case BMP348_COM_I2C:
            Wire.beginTransmission(i2cAddress);
            
            // Registers have to be written in address/data pairs
            for(uint8_t i = 0; i < numBytes; i++)
            {
                Wire.write(regAddress + i);
                Wire.write(((uint8_t*) data)[i]);
            }

            Wire.endTransmission();
            break;
        
        case BMP348_COM_SPI:
            SPI.beginTransaction(spiSettings);
            digitalWrite(spiCSPin, LOW);
            
            // Registers have to be written in address/data pairs
            for(uint8_t i = 0; i < numBytes; i++)
            {
                SPI.transfer(regAddress + i);
                SPI.transfer(((uint8_t*) data)[i]);
            }

            digitalWrite(spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }
}