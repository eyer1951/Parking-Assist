# ESP32 Source Code

The code was written in C++ using the PlatformIO development environment running in Visual Studio Code. 
There are four main files in the source:

* main.css – the main program, including setup() and loop().
 
* main.h – a header file defining some project-related items used by the project.css file, and defining the array of parameters used.

* project.css – code implementing functions common to many projects, including:

  *	NVS-related functionality including initialization, saving and restoring parameters
  *	Some helpful utility functions
  * mDNS-related functionality
  *	WiFi connection functions, including reconnection upon lost connection
  *	Over-the-air (OTA) code download functionality
  *	WebSockets functionality supporting the parameter array approach

* project.h – header file for the project, providing procedure declarations and definition of a Ring buffer class.

Library dependencies are:

* adafruit/Adafruit Unified Sensor, version 1.1.4 or above

* adafruit/DHT sensor library version 1.4.1 or above

*	arduino-libraries/NTPClient version 3.1.0 or above

*	m5ez/ezTime version 0.8.3 or above

*	fastled/FastLED version 3.4.0
