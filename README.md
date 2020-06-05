# ContinuousThermometer

Implementation on the ESP32 board of a temperature processing model for the MLX90614 and MCP9808 sensors, in order to obtain a reliable continuous temperature measurement. The main motivation for this project was the COVID-19 pandemic and the need for a low-cost thermometer for continuous measurement of body temperature.

This repository contains the following files, to be implemented in Arduino IDE:

      - MLX90614 IR sensor with continuous temperature
      
      - MMCP9808 IC sensor with continuous temperature
      
      - MLC90614 IR and MMCP9808 IC sensor with continuous temperature, integrated in one script
      
      - MLX90614 IR sensor with continuous and punctual temperature
      
      - MMCP9808 IC sensor with continuous and punctual temperature
      
      - Controlled-Temperature Ambient ('Black Box') for IR and IC stability
      
      - ESP32 Firebase integration for IR Sensor (Example)
	
For this measurements, one must assemble the following Breadboard montage:

![breadboard](https://user-images.githubusercontent.com/55759937/83919298-383f3c00-a772-11ea-8389-b60e2be4441f.png)
