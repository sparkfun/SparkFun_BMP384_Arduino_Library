#include <SPI.h>
#include "SparkFunBMP384.h"

BMP384 pressureSensor;

uint8_t csPin = 10;

void setup()
{
    Serial.begin(115200);
    Serial.println("BMP384 example begin!");

    SPI.begin();

    // Check if sensor is connected
    while(!pressureSensor.beginSPI(csPin))
    {
        // Not connected, inform user
        Serial.println("Error: BMP384 not connected, check wiring!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMP384 connected!");
}

void loop()
{
    // Print temperature and pressure every second
    Serial.print("Temperature (C): ");
    Serial.print(pressureSensor.getTemperature());
    
    Serial.print("\t\t");

    Serial.print("Pressure (Pa): ");
    Serial.println(pressureSensor.getPressure());

    delay(1000);
}