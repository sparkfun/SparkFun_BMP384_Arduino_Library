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

    // Convert to temperature in C
    return convertTemperature(rawData);
}

float BMP384::getPressure()
{
    // Get raw data from sensor
    uint64_t rawData = getRawData();

    // Convert to temperature in C
    return convertPressure(rawData);
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

float BMP384::convertTemperature(uint64_t rawData)
{
    // Middle 3 bytes are raw temperature
    uint32_t rawTemp = (rawData >> 24) & 0xFFFFFF;

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

float BMP384::convertPressure(uint64_t rawData)
{
    // Need to get true temperature first
    float trueTemp = convertTemperature(rawData);

    // Lowest 3 bytes are raw pressure
    uint32_t rawPress = rawData & 0xFFFFFF;

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

void BMP384::readRegisters(uint8_t regAddress, void* dataBuffer, uint8_t numBytes)
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
            // Write all requested bytes at these registers
            Wire.beginTransmission(i2cAddress);
            Wire.write(regAddress);
            Wire.write((uint8_t*) data, numBytes);
            Wire.endTransmission();
            break;
        
        case BMP348_COM_SPI:
            SPI.beginTransaction(spiSettings);
            digitalWrite(spiCSPin, LOW);
            
            // Registers have to be written one by one
            // for(uint8_t i = 0; i < numBytes; i++)
            // {
            //     SPI.transfer(regAddress);
            //     SPI.transfer((uint8_t*) data, numBytes);
            //     regAddress++;
            // }

            SPI.transfer(regAddress);
            SPI.transfer((uint8_t*) data, numBytes);

            digitalWrite(spiCSPin, HIGH);
            SPI.endTransaction();
            break;
    }
}