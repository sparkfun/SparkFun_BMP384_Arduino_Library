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

    // The oversampling (OSR) multipliers should be a power of 2, with a maximum of 32 (2^5).
    // At 1x OSR, we get 16-bit resolution from each sensor (0.005C and 2.64Pa).
    // At 32x OSR, we get 21-bit resolution from each sensor (0.00015C and 0.085Pa).
    pressureSensor.setTemperatureOSRMultiplier(1);
    pressureSensor.setPressureOSRMultiplier(1);

    /////////////////////////////////////////////////////////////////
    //                           WARNING                           //
    //                                                             //
    // As of this time, setting any value higher than 1 causes     //
    // sensor values to stop changing. The cause and solution are  //
    // not yet known, this is being worked on!                     //
    /////////////////////////////////////////////////////////////////
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