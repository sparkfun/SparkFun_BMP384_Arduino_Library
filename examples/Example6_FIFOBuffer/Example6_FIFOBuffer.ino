#include <Wire.h>
#include "SparkFunBMP384.h"

// Create a new sensor object
BMP384 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP384_I2C_ADDRESS_DEFAULT; // 0x77
//uint8_t i2cAddress = BMP384_I2C_ADDRESS_SECONDARY; // 0x76

// Pin used for interrupt detection
int interruptPin = 2;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

// Create a buffer for FIFO data
// Note - on some systems (eg. Arduino Uno), warnings will be generated
// when numSamples is large (eg. >= 5)
const uint8_t numSamples = 5;
bmp3_data fifoData[numSamples];

// Track FIFO length to give progress updates
uint8_t previousFIFOLength = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP384 Example6 begin!");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x77)
    while(pressureSensor.beginI2C(i2cAddress) != BMP3_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP384 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMP384 connected!");

    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // By default, the BMP384 samples at 200Hz. However we don't want interrupts to
    // trigger that fast in this example, so we can lower the output data rate (ODR)
    // with this function. In this case, we're setting it to 1.5Hz. For all possible
    // ODR settings, see bmp3_defs.h (line 232-249)
    err = pressureSensor.setODRFrequency(BMP3_ODR_1_5_HZ);
    if(err != BMP3_OK)
    {
        // Setting ODR failed, most likely an invalid frequncy (code -3)
        Serial.print("ODR setting failed! Error code: ");
        Serial.println(err);
    }

    // Configure the FIFO buffer to store pressure and temperature data
    // Note - this is also where the FIFO interrupt conditions are set
    // Note - this must be called before any other FIFO functions
    bmp3_fifo_settings fifoSettings =
    {
        .mode            = BMP3_ENABLE,  // Enable the FIFO buffer
        .stop_on_full_en = BMP3_DISABLE, // Stop writing to FIFO once full, or overwrite oldest data
        .time_en         = BMP3_DISABLE, // Enable sensor time, only 1 frame at end of buffer
        .press_en        = BMP3_ENABLE,  // Enable pressure sensor recording
        .temp_en         = BMP3_ENABLE,  // Enable temperature sensor recording
        .down_sampling   = BMP3_FIFO_NO_SUBSAMPLING, // Set downsampling factor
        .filter_en       = BMP3_DISABLE, // Enable data filtering
        .fwtm_en         = BMP3_ENABLE,  // Trigger interrupt on FIFO watermark
        .ffull_en        = BMP3_DISABLE  // Trigger interrupt on FIFO full
    };
    err = pressureSensor.setFIFOSettings(fifoSettings);
    if(err != BMP3_OK)
    {
        // FIFO settings failed, most likely a communication error (code -2)
        Serial.print("FIFO settings failed! Error code: ");
        Serial.println(err);
    }

    // Set watermark to number of samples we want
    err = pressureSensor.setFIFOWatermark(numSamples);
    if(err != BMP3_OK)
    {
        // Setting watermark failed, most likely too many samples (code 2)
        Serial.print("Watermark setting failed! Error code: ");
        Serial.println(err);
    }

    // Configure the BMP384 to trigger interrupts when the FIFO watermark has been
    // reached. Note - setInterruptSettings() only sets the interrupt behavior and
    // the data ready interrupt condition. The FIFO interrupt conditions must be
    // set with setFIFOSettings()
    bmp3_int_ctrl_settings interruptSettings =
    {
        .output_mode = BMP3_INT_PIN_PUSH_PULL,   // Push-pull or open-drain
        .level       = BMP3_INT_PIN_ACTIVE_HIGH, // Active low or high
        .latch       = BMP3_INT_PIN_NON_LATCH,   // Latch or non-latch
        .drdy_en     = BMP3_DISABLE              // Trigger interrupts when data is ready
    };
    err = pressureSensor.setInterruptSettings(interruptSettings);
    if(err != BMP3_OK)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Interrupt settings failed! Error code: ");
        Serial.println(err);
    }

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), bmp384InterruptHandler, RISING);
}

void loop()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Get number of data samples currently stored in FIFO buffer
    uint8_t currentFIFOLength = 0;
    err = pressureSensor.getFIFOLength(&currentFIFOLength);
    if(err != BMP3_OK)
    {
        // FIFO length failed, most likely a communication error (code -2)
        Serial.print("FIFO length failed! Error code: ");
        Serial.println(err);

        // If getFIFOLength() failed this time, it will most likely fail next time. So
        // let's wait a bit before trying again
        delay(1000);
    }

    // Check whether number of samples in FIFO buffer has changed
    if(previousFIFOLength != currentFIFOLength)
    {
        // Update FIFO length
        previousFIFOLength = currentFIFOLength;

        // Print current FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(currentFIFOLength);
        Serial.print("/");
        Serial.println(numSamples);

        // If the buffer length goes beyond the watermark level, then an
        // interrupt was missed. This example will likely run into issues,
        // so we'll just clear the FIFO buffer
        if(currentFIFOLength > numSamples)
        {
            Serial.println("Too many samples in FIFO buffer, flushing...");
            
            err = pressureSensor.flushFIFO();
            if(err != BMP3_OK)
            {
                // FIFO flush failed, most likely a communication error (code -2)
                Serial.print("FIFO flush failed! Error code: ");
                Serial.println(err);
            }
        }
    }

    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.println("Interrupt occurred!");

        // Get the interrupt status to know which condition triggered
        bmp3_int_status interruptStatus;
        err = pressureSensor.getInterruptStatus(&interruptStatus);
        if(err != BMP3_OK)
        {
            // Status get failed, most likely a communication error (code -2)
            Serial.print("Get interrupt status failed! Error code: ");
            Serial.println(err);
            return;
        }

        // Make sure this is the "FIFO watermerk" interrupt condition
        if(interruptStatus.fifo_wm)
        {
            // Get FIFO data from the sensor
            err = pressureSensor.getFIFOData(fifoData, numSamples);
            if(err < BMP3_OK)
            {
                // FIFO data get failed, most likely a communication error (code -2)
                Serial.print("Get FIFO data failed! Error code: ");
                Serial.println(err);
                return;
            }
            if(err > BMP3_OK)
            {
                // FIFO data get warning, most likely min/max pressure/temperature (codes 3/4/5/6)
                // This is likely to occur on some systems (eg. Arduino Uno)
                // when numSamples is large (eg. >= 5)
                Serial.print("Get FIFO data warning! Error code: ");
                Serial.println(err);
            }

            // Data was acquired successfully, print it all out
            for(uint8_t i = 0; i < numSamples; i++)
            {
                Serial.print("Sample number: ");
                Serial.print(i);
                Serial.print("\t\t");
                Serial.print("Temperature (C): ");
                Serial.print(fifoData[i].temperature);
                Serial.print("\t\t");
                Serial.print("Pressure (Pa): ");
                Serial.println(fifoData[i].pressure);
            }
        }
        else
        {
            Serial.println("Wrong interrupt condition!");
        }
    }
}

void bmp384InterruptHandler()
{
    interruptOccurred = true;
}