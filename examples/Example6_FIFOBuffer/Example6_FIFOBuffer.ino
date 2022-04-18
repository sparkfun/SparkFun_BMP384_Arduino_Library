#include <Wire.h>
#include "SparkFunBMP384.h"

BMP384 pressureSensor;

int interruptPin = 2;

volatile bool interruptOccurred = false;

const uint16_t numSamples = 5;
float tempData[numSamples] = {0};
float pressData[numSamples] = {0};

uint8_t numFIFOSamples = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("BMP384 example begin!");

    Wire.begin();

    // Uncomment to change I2C address from default
    //pressureSensor.setI2CAddress(BMP384_I2C_ADDRESS_0);

    // Check if sensor is connected
    while(!pressureSensor.beginI2C())
    {
        // Not connected, inform user
        Serial.println("Error: BMP384 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMP384 connected!");

    // By default, the BMP384 samples every 5ms. We can set the ODR prescaler
    // to slow down the output data rate. The prescaler should be a power of 2,
    // with a maximum of 2^17. We can use a prescaler of 256 to get measurements
    // every 1.28 seconds
    pressureSensor.setODRPrescaler(256);

    // Configure the FIFO buffer to store pressure and temperature data
    BMP384_FIFOConfig fifoConfig =
    {
        .flags =
        {
            .fifoEnable = true,  // Enable the FIFO buffer
            .stopOnFull = false, // Stop writing to FIFO once full, or overwrite oldest data
            .timeEnable = false, // Enable sensor time, only 1 frame at end of buffer
            .pressEnable = true, // Enable pressure sensor recording
            .tempEnable = true,  // Enable temperature sensor recording
            .subsampling = 0,    // Set subsampling, actually 2^subsampling
            .dataSelect = 0      // Unfiltered (0) or filtered (1) data
        }
    };
    pressureSensor.setFIFOConfig(fifoConfig);

    // Set FIFO watermark to desired number of samples
    pressureSensor.setFIFOWatermarkSamples(numSamples);

    // Set an interrupt to occur once the watermark has been reached
    BMP384_InterruptConfig interruptConfig =
    {
        .flags =
        {
            .openDrain     = false,
            .activeHigh    = true,
            .latch         = false,
            .fifoWatermark = true,
            .fifoFull      = false,
            .dataReady     = false
        }
    };
    pressureSensor.setInterruptConfig(interruptConfig);

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), bmp384InterruptHandler, RISING);

    // Changing the FIFO config causes extra data to be stored in the FIFO
    // buffer. We don't care about these bytes, and they add to the watermark,
    // so we can get rid of them by flushing the buffer
    pressureSensor.flushFIFO();
}

void loop()
{
    // Wait for number of samples in FIFO buffer to change
    if(numFIFOSamples != pressureSensor.getFIFOLengthSamples())
    {
        // Update number of samples
        numFIFOSamples = pressureSensor.getFIFOLengthSamples();

        // Print FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(numFIFOSamples);
        Serial.print("/");
        Serial.println(numSamples);
    }

    // Check whether interrupt occurred 
    if(interruptOccurred)
    {
        // Make sure this is the "data ready" interrupt condition
        BMP384_InterruptStatus status = pressureSensor.getInterruptStatus();
        if(status.flags.fifoWatermark)
        {
            // Read out FIFO data
            Serial.println("FIFO watermark reached! Reading FIFO data...");
            uint8_t numDataRead = pressureSensor.readFIFO(tempData, pressData, numSamples);

            // Check whether all data was acquired from FIFO buffer
            if(numDataRead != numSamples)
            {
                Serial.println("Warning: not all data acquired from FIFO!");
            }

            // Print out all FIFO data
            for(uint8_t i = 0; i < numDataRead; i++)
            {
                Serial.print("Temperature (C): ");
                Serial.print(tempData[i]);
                
                Serial.print("\t\t");

                Serial.print("Pressure (Pa): ");
                Serial.println(pressData[i]);
            }
        }

        // Reset flag for next interrupt
        interruptOccurred = false;
    }
}

void bmp384InterruptHandler()
{
    interruptOccurred = true;
}