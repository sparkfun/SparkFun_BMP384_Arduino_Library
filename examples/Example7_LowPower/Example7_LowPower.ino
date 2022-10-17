#include <Wire.h>
#include "SparkFunBMP384.h"

// Create a new sensor object
BMP384 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP384_I2C_ADDRESS_DEFAULT; // 0x77
//uint8_t i2cAddress = BMP384_I2C_ADDRESS_SECONDARY; // 0x76

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP384 Example7 begin!");

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

    // Calling beginI2C() puts the sensor into normal mode, which has
    // relatively high power consumption. We can put the sensor into sleep
    // mode to minimize this.
    err = pressureSensor.setMode(BMP3_MODE_SLEEP);
    if(err != BMP3_OK)
    {
        // Set mode failed, most likely an invalid mode (code -5)
        Serial.print("Set mode failed! Error code: ");
        Serial.println(err);
    }
}

void loop()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Wait until next measurement. For low power applications, this could be
    // replaced by setting the microcontroller into a sleep state
    delay(1000);

    // Transition from sleep mode into forced mode. This will trigger a single
    // measurement, after which the sensor automatically returns to sleep mode
    // Note - the sensor can only enter forced mode from sleep mode. Transitions
    // between forced and normal modes are ignored
    err = pressureSensor.setMode(BMP3_MODE_FORCED);
    if(err != BMP3_OK)
    {
        // Set mode failed, most likely a communication error (code -2)
        Serial.print("Set mode failed! Error code: ");
        Serial.println(err);
    }

    // Wait for measurement, with a timeout period just in case
    bool dataReady = false;
    uint32_t t0 = millis();
    uint32_t timeout = 1000;
    do
    {
        // Check whether timeout has occurred
        if((millis() - t0) > timeout)
        {
            // Measurement has taken too long to finish. There's likely a
            // problem somewhere, so we'll just abort this measurement
            Serial.println("Measurement timeout occurred!");
            return;            
        }

        // Get sensor status
        bmp3_sens_status sensorStatus;
        err = pressureSensor.getSensorStatus(&sensorStatus);
        if(err != BMP3_OK)
        {
            // Get status failed, most likely a communication error (code -2)
            Serial.print("Get status failed! Error code: ");
            Serial.println(err);

            // If we got an error, it will most likely persist, so just exit
            return;
        }

        // Determine whether measurement has completed
        dataReady = sensorStatus.drdy_press || sensorStatus.drdy_temp;
    } while (!dataReady);
    
    // Measurement has finished, let's grab the data
    bmp3_data data;
    err = pressureSensor.getSensorData(&data);

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