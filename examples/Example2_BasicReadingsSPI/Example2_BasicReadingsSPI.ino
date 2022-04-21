#include <SPI.h>
#include "SparkFunBMP384.h"

// Create a new sensor object
BMP384 pressureSensor;

// SPI parameters
uint8_t csPin = 10;
uint32_t clockFrequency = 100000;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP384 example begin!");

    // Initialize the SPI library
    SPI.begin();

    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    while(pressureSensor.beginSPI(csPin, clockFrequency) != BMP3_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP384 not connected, check wiring and CS pin!");

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