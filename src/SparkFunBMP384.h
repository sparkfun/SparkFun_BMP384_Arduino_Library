#ifndef __BMP384_H__
#define __BMP384_H__

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

// Device registers
#define BMP384_REG_CHIP_ID          0x00
#define BMP384_REG_ERR_REG          0x02
#define BMP384_REG_STATUS           0x03
#define BMP384_REG_DATA0            0x04
#define BMP384_REG_DATA1            0x05
#define BMP384_REG_DATA2            0x06
#define BMP384_REG_DATA3            0x07
#define BMP384_REG_DATA4            0x08
#define BMP384_REG_DATA5            0x09
#define BMP384_REG_SENSORTIME0      0x0C
#define BMP384_REG_SENSORTIME1      0x0D
#define BMP384_REG_SENSORTIME2      0x0E
#define BMP384_REG_SENSORTIME3      0x0F
#define BMP384_REG_EVENT            0x10
#define BMP384_REG_INT_STATUS       0x11
#define BMP384_REG_FIFO_LENGTH_0    0x12
#define BMP384_REG_FIFO_LENGTH_1    0x13
#define BMP384_REG_FIFO_DATA        0x14
#define BMP384_REG_FIFO_WTM_0       0x15
#define BMP384_REG_FIFO_WTM_1       0x16
#define BMP384_REG_FIFO_CONFIG_1    0x17
#define BMP384_REG_FIFO_CONFIG_2    0x18
#define BMP384_REG_INT_CTRL         0x19
#define BMP384_REG_IF_CONF          0x1A
#define BMP384_REG_PWR_CTRL         0x1B
#define BMP384_REG_OSR              0x1C
#define BMP384_REG_ODR              0x1D
#define BMP384_REG_CONFIG           0x1F
#define BMP384_REG_CMD              0x7E

// Device calibration registers
#define BMP384_REG_NVM_PAR_T1_LSB   0x31
#define BMP384_REG_NVM_PAR_T1_MSB   0x32
#define BMP384_REG_NVM_PAR_T2_LSB   0x33
#define BMP384_REG_NVM_PAR_T2_MSB   0x34
#define BMP384_REG_NVM_PAR_T3       0x35
#define BMP384_REG_NVM_PAR_P1_LSB   0x36
#define BMP384_REG_NVM_PAR_P1_MSB   0x37
#define BMP384_REG_NVM_PAR_P2_LSB   0x38
#define BMP384_REG_NVM_PAR_P2_MSB   0x39
#define BMP384_REG_NVM_PAR_P3       0x3A
#define BMP384_REG_NVM_PAR_P4       0x3B
#define BMP384_REG_NVM_PAR_P5_LSB   0x3C
#define BMP384_REG_NVM_PAR_P5_MSB   0x3D
#define BMP384_REG_NVM_PAR_P6_LSB   0x3E
#define BMP384_REG_NVM_PAR_P6_MSB   0x3F
#define BMP384_REG_NVM_PAR_P7       0x40
#define BMP384_REG_NVM_PAR_P8       0x41
#define BMP384_REG_NVM_PAR_P9_LSB   0x42
#define BMP384_REG_NVM_PAR_P9_MSB   0x43
#define BMP384_REG_NVM_PAR_P10      0x44
#define BMP384_REG_NVM_PAR_P11      0x45

// Device values
#define BMP384_CHIP_ID              0x50
#define BMP384_I2C_ADDRESS_0        0x76
#define BMP384_I2C_ADDRESS_1        0x77

// CMD register commands
#define BMP384_CMD_FIFO_FLUSH       0xB0
#define BMP384_CMD_RESET            0xB6

// Power modes
#define BMP384_POWER_MODE_SLEEP     0b00
#define BMP384_POWER_MODE_FORCED    0b01
#define BMP384_POWER_MODE_NORMAL    0b11

// Possible communication interfaces
#define BMP348_COM_I2C              0
#define BMP348_COM_SPI              1

// Raw calibration values, acquired from the sensor
struct BMP384_CalibrationRaw
{
    // Temperature values
    uint16_t NVM_PAR_T1;
    uint16_t NVM_PAR_T2;
    int8_t   NVM_PAR_T3;
    
    // Pressure values
    int16_t  NVM_PAR_P1;
    int16_t  NVM_PAR_P2;
    int8_t   NVM_PAR_P3;
    int8_t   NVM_PAR_P4;
    uint16_t NVM_PAR_P5;
    uint16_t NVM_PAR_P6;
    int8_t   NVM_PAR_P7;
    int8_t   NVM_PAR_P8;
    int16_t  NVM_PAR_P9;
    int8_t   NVM_PAR_P10;
    int8_t   NVM_PAR_P11;
} __attribute__((packed)); // Prevent padding, this will be read directly into within getCalibrationData()

// Calibration values after converting, as per datasheet's equations
struct BMP384_Calibration
{
    // Temperature values
    float PAR_T1;
    float PAR_T2;
    float PAR_T3;
    
    // Pressure values
    float PAR_P1;
    float PAR_P2;
    float PAR_P3;
    float PAR_P4;
    float PAR_P5;
    float PAR_P6;
    float PAR_P7;
    float PAR_P8;
    float PAR_P9;
    float PAR_P10;
    float PAR_P11;
};

// Flags used to configured the INT_CTRL register
union BMP384_InterruptConfig
{
    struct
    {
        uint8_t openDrain     : 1;
        uint8_t activeHigh    : 1;
        uint8_t latch         : 1;
        uint8_t fifoWatermark : 1;
        uint8_t fifoFull      : 1;
        uint8_t               : 1; // Padding
        uint8_t dataReady     : 1;
    } flags;
    uint8_t registerVal;
};

// Flags returned by INT_STATUS register
union BMP384_InterruptStatus
{
    struct
    {
        uint8_t fifoWatermark : 1;
        uint8_t fifoFull      : 1;
        uint8_t               : 1; // Padding
        uint8_t dataReady     : 1;
    } flags;
    uint8_t registerVal;
};

// Flags used to configured the FIFO buffer
union BMP384_FIFOConfig
{
    struct
    {
        uint8_t fifoEnable        : 1;
        uint8_t stopOnFull        : 1;
        uint8_t timeEnable        : 1;
        uint8_t pressEnable       : 1;
        uint8_t tempEnable        : 1;
        uint8_t                   : 3; // Padding
        uint8_t subsampling       : 3;
        uint8_t dataSelect        : 2;
    } flags;
    uint16_t registerVals;
};

class BMP384
{
    public:
        // Constructor
        BMP384();

        // Initialization methods
        bool begin();
        bool beginI2C(uint8_t address = BMP384_I2C_ADDRESS_1);
        bool beginSPI(uint8_t csPin);
        
        // Chip control
        void reset();
        void setPowerMode(uint8_t modeBits);
        void enableTemperature(bool enable);
        void enablePressure(bool enable);

        // Returns measured temperature in C
        float getTemperature();
        
        // Returns measured pressure in Pa
        float getPressure();

        // Chip ID, should return BMP384_CHIP_ID
        uint8_t getChipID();

        // Change the address of this board
        bool setI2CAddress(uint8_t address);

        // Interrupt configuration
        BMP384_InterruptConfig getInterruptConfig();
        void setInterruptConfig(BMP384_InterruptConfig config);

        // Interrupt status
        BMP384_InterruptStatus getInterruptStatus();

        // Output data rate (ODR) config
        // Prescaler should be a power of 2, max of 131075 (2^17)
        uint32_t getODRPrescaler();
        void setODRPrescaler(uint32_t prescaler);

        // IIR filter config
        // Coefficient should be a power of 2 minus 1, max of 127 (2^7 - 1)
        uint8_t getFilterCoefficient();
        void setFilterCoefficient(uint8_t coefficient);

        // Oversampling (OSR) config
        // Multiplier should be a power of 2, max of 32 (2^5)
        uint8_t getTemperatureOSRMultiplier();
        uint8_t getPressureOSRMultiplier();
        void setTemperatureOSRMultiplier(uint8_t multiplier);
        void setPressureOSRMultiplier(uint8_t multiplier);

        // FIFO buffer config
        void setFIFOConfig(BMP384_FIFOConfig config);
        void setFIFOWatermarkBytes(uint16_t numBytes);
        void setFIFOWatermarkSamples(uint8_t numSamples);
        uint16_t getFIFOLengthBytes();
        uint8_t getFIFOLengthSamples();
        int16_t readFIFO(float* tempData, float* pressData, uint16_t numData);
        void flushFIFO();

    private:
        // Calibration
        BMP384_Calibration calibrationData;
        void getCalibrationData();

        // Returns raw values from the 6 data registers.
        // Lowest 3 bytes are pressure, next 3 bytes are temperature.
        // Highest 2 bytes of uint64_t are zeros
        uint64_t getRawData();

        // Converts value from getRawData() to temperature in C
        float convertTemperature(uint32_t rawTemp);

        // Converts value from getRawData() to pressure in Pa
        float convertPressure(uint32_t rawPress, float trueTemp);

        // Methods to read/write registers
        uint8_t readRegister(uint8_t regAddress);
        void readRegisters(uint8_t regAddress, void* dataBuffer, uint16_t numBytes);
        void writeRegister(uint8_t regAddress, uint8_t data);
        void writeRegisters(uint8_t regAddress, void* dataBuffer, uint8_t numBytes);

        // Communication interface (I2C or SPI)
        uint8_t communicationInterface = BMP348_COM_I2C;

        // Default I2C address
        uint8_t i2cAddress = BMP384_I2C_ADDRESS_1;

        // SPI chip select pin
        uint8_t spiCSPin;

        // SPI bus settings
        SPISettings spiSettings{1000000, MSBFIRST, SPI_MODE0};

        // Expected number of bytes per FIFO frame, used for watermark
        uint16_t numBytesPerFIFOFrame = 1;

        // Register values to remember, with default values
        uint8_t  regPwrCtrl    = 0x00;
        uint8_t  regOsr        = 0x00; // Datasheet says this should be 0x02, but it's not
        uint16_t regFIFOConfig = 0x0202;
};

#endif