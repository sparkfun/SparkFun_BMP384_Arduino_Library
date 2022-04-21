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
    Serial.println("BMP384 example begin!");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x77)
    while(pressureSensor.beginI2C(i2cAddress) != BMP3_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP384 not connected, check wiring and I2C address!");
        Serial.println(i2cAddress);

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMP384 connected!");
}

void loop()
{
    // Get measurements from the sensor
    bmp3_data data = pressureSensor.getSensorData();

    // Print temperature and pressure
    Serial.print("Temperature (C): ");
    Serial.print(data.temperature);
    
    Serial.print("\t\t");

    Serial.print("Pressure (Pa): ");
    Serial.println(data.pressure);

    // Only print every second
    delay(1000);
}