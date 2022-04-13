#include <Wire.h>
#include "SparkFunBMP384.h"

BMP384 pressureSensor;

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