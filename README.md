# Mbed Lidar Lite Driver
Garmin Lidar Lite v3 Mbed driver.

# Table Of Contents
- [Overview](#overview)

# Overview
Source code in [`main.cpp`](./main.cpp) provides a functions used to communicate
with a [Garmin Lidar Lite V3 sensor](https://www.adafruit.com/product/4058).

See the `main()` method in this file for an example usage.

The Mbed platform is required. Although only for I2C serial communication, the
source code could easily be adapted to use any other platform which provides
an I2C API.
