SparkFun BMP384 Arduino Library
========================================
<table class="table table-hover table-striped table-bordered">
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/19662"><img src="https://cdn.sparkfun.com/assets/parts/1/9/3/3/8/19662-SparkFun_Pressure_Sensor_-_BMP384__Qwiic_-01.jpg" alt="SparkFun Pressure Sensor - BMP384 (Qwiic)"></a></td>
    <td><a href="https://www.sparkfun.com/products/19833"><img src="https://cdn.sparkfun.com/assets/learn_tutorials/2/4/2/0/Qwiic_Micro_BMP384-Thumb.jpg" alt="SparkFun Micro Pressure Sensor - BMP384 (Qwiic)"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/19662"><i>SparkFun Pressure Sensor - BMP384 (Qwiic)</i></a></td>
    <td><a href="https://www.sparkfun.com/products/19833"><i>SparkFun Micro Pressure Sensor - BMP384 (Qwiic)</i></a></td>
  </tr>
</table>

This library provides an easy way to control the BMP384 pressure sensor. It is a very precise sensor, allowing up to 21 bits of precision with oversampling, plus a built-in filter to reduce noise. That's sufficient to measure the difference in pressure of raising/lowering the sensor by just a few inches!

The sensor also includes a temperature sensor, however it's not really intended for measuring air temperature; its main purpose is to compensate the pressure measurements. The measured temperature is usually a few degrees above ambient, and depends on things like the PCB temperature, sensor element, and even the I2C/SPI clock speed!

This library implements [Bosch's BMP3 API](https://github.com/BoschSensortec/BMP3-Sensor-API) in an Arduino-friendly way. All functions return error codes to indicate the result of each operation, where BMP3_OK (0) indicates success. The examples demonstrate basic ways of handling these errors, but error checking can be omitted to reduce code clutter if desired.

## Repository Contents
* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.

## Documentation
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Hardware Repo](https://github.com/sparkfun/SparkFun_Qwiic_Pressure_Sensor_BMP384)** - Repository for the BMP384 board.
* **[Library](https://github.com/sparkfun/SparkFun_BMP384_Arduino_Library)** - This library, providing functions to write applications for the BMP384 with Arduino IDE.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/qwiic-pressure-sensor-bmp384-hookup-guide)** - Basic hookup guide for the BMP384.
* **[LICENSE.md](./LICENSE.md)** - License Information

## Product Versions
* [SEN-19662](https://www.sparkfun.com/products/19662) - BMP384 Qwiic Board

## Version History

* [v1.0.0](https://github.com/sparkfun/SparkFun_BMP384_Arduino_Library/releases/tag/v1.0.0) - Initial public release.

## License Information

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

_<COLLABORATION CREDIT>_
