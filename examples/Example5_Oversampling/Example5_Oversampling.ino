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
    Serial.println("BMP384 Example5 begin!");

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

    // Both the temperature and pressure sensors support oversampling, where
    // multiple measurements are performed to increase resolution and decrease
    // noise. By default, no oversampling is used.
    // At 1x OSR, we get 16-bit resolution from each sensor (0.005C and 2.64Pa).
    // At 32x OSR, we get 21-bit resolution from each sensor (0.00015C and 0.085Pa).
    bmp3_odr_filter_settings osrMultipliers =
    {
        .press_os = BMP3_OVERSAMPLING_32X,
        .temp_os = BMP3_OVERSAMPLING_2X,
        0,0 // Unused values, included to avoid compiler warnings-as-error
    };
    err = pressureSensor.setOSRMultipliers(osrMultipliers);
    if(err)
    {
        // Setting OSR failed, most likely an invalid multiplier (code -3)
        Serial.print("Error setting OSR! Error code: ");
        Serial.println(err);
    }

    // Note - increasing oversampling also requires ODR to be increased.
    // setOSRMultipliers() will automatically increase the ODR if needed, but
    // will still return BMP3_OK. You can check whether it was changed by
    // calling getODRFrequency()
    uint8_t odr = 0;
    err = pressureSensor.getODRFrequency(&odr);
    if(err)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Error getting ODR! Error code: ");
        Serial.println(err);
    }

    // The true ODR frequency in Hz is [200 / (2^odr)]
    Serial.print("ODR Frequency: ");
    Serial.print(200 / pow(2, odr));
    Serial.println("Hz");

    // Give user time to read ODR
    delay(1000);
}

void loop()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP3_OK;

    // Get sensor status
    bmp3_sens_status sensorStatus;
    err = pressureSensor.getSensorStatus(&sensorStatus);
    if(err)
    {
        // Get status failed, most likely a communication error (code -2)
        Serial.print("Get status failed! Error code: ");
        Serial.println(err);
    }

    // Check whether sensor data is ready
    if(sensorStatus.drdy_press || sensorStatus.drdy_temp)
    {
        // Measurement is complete, get measurements from the sensor
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
}