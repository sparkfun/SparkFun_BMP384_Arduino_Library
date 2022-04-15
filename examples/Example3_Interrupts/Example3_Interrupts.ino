#include <Wire.h>
#include "SparkFunBMP384.h"

BMP384 pressureSensor;

int interruptPin = 2;

volatile bool interruptOccurred = false;

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

    // Configure the BMP384 to trigger interrupts whenever a measurement is performed
    BMP384_InterruptConfig config =
    {
        .flags =
        {
            .openDrain     = false, // Open-drain or push/pull
            .activeHigh    = true,  // Active high or low signal level
            .latch         = false, // Prevent interrupt from automatically clearing
            .fifoWatermark = false, // Trigger interrupt when FIFO watermark is reached
            .fifoFull      = false, // Trigger interrupt when FIFO is full
            .dataReady     = true   // Trigger interrupt when measurement is complete
        }
    };
    pressureSensor.setInterruptConfig(config);

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), bmp384InterruptHandler, RISING);
}

void loop()
{
    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Make sure this is the "data ready" interrupt condition
        BMP384_InterruptStatus status = pressureSensor.getInterruptStatus();
        if(status.flags.dataReady)
        {
            // Print the current temperature and pressure
            Serial.print("Temperature (C): ");
            Serial.print(pressureSensor.getTemperature());
            
            Serial.print("\t\t");

            Serial.print("Pressure (Pa): ");
            Serial.println(pressureSensor.getPressure());
        }

        // Reset flag for next interrupt
        interruptOccurred = false;
    }
}

void bmp384InterruptHandler()
{
    interruptOccurred = true;
}