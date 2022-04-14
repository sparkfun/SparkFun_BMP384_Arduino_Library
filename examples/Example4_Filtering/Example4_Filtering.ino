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

    // By default, the filter coefficient is set to 0 (no filtering). It can be
    // set to any power of 2 minus 1, with a max of 127 (2^7 - 1), which causes
    // the measurements to be smoothed out
    pressureSensor.setFilterCoefficient(127);
}

void loop()
{
    // Print temperature and pressure every 100ms (faster than normal to see effects)
    Serial.print("Temperature (C): ");
    Serial.print(pressureSensor.getTemperature());
    
    Serial.print("\t\t");

    Serial.print("Pressure (Pa): ");
    Serial.println(pressureSensor.getPressure());

    delay(100);
}