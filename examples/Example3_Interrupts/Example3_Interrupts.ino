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

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP384 Example3 begin!");

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

    // Configure the BMP384 to trigger interrupts whenever a measurement is performed
    bmp3_int_ctrl_settings interruptSettings =
    {
        .output_mode = BMP3_INT_PIN_PUSH_PULL,   // Push-pull or open-drain
        .level       = BMP3_INT_PIN_ACTIVE_HIGH, // Active low or high
        .latch       = BMP3_INT_PIN_NON_LATCH,   // Latch or non-latch
        .drdy_en     = BMP3_ENABLE               // Trigger interrupts when data is ready
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
    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.print("Interrupt occurred!\t\t");

        // Variable to track errors returned by API calls
        int8_t err = BMP3_OK;

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

        // Make sure this is the "data ready" interrupt condition
        if(interruptStatus.drdy)
        {
            // Get measurements from the sensor
            bmp3_data data;
            int8_t err = pressureSensor.getSensorData(&data);

            // Check whether data was acquired successfully
            if(err == BMP3_OK)
            {
                // Acquisistion succeeded, print temperature and pressure
                Serial.print("Temperature (C): ");
                Serial.print(data.temperature);
                Serial.print("\t\t");
                Serial.print("Pressure (Pa): ");
                Serial.println(data.pressure);
            }
            else
            {
                // Acquisition failed, most likely a communication error (code -2)
                Serial.print("Error getting data from sensor! Error code: ");
                Serial.println(err);
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