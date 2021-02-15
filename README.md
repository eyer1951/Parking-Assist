# Parking-Assist
ESP32-based parking assistant using ultrasonic distance measurement and a multi-LED display
This project was an opportunity to combine several hardware/software technologies to create something I use every day. 
The Parking Assist unit used an HC-SR04 ultrasonic distance sensor to help signal the precise positioning of the car 
as it is moved into the garage. The unit was mounted on the side of a file cabinet in front of my parking space.
Here's a view from inside the car as the driver pulls in:

![](images/parkme-540p.gif)

Technologies, in addition to the use of the HC-SR04, include:

*	Control of an addressable (WS2812B) LED strip via the FastLED library

*	Sensing the ambient air temperature to compute an accurate speed of sound for distance measurement

*	WiFi connection to the home network for control

*	Setting the “target” parking distance via pushbutton

*	Setup of parameters by web application

*	Web application implemented an approach where the characteristics of the set of control parameters were defined in a JSON file

*	Websocket interface for delivery of parameters

*	A telnet interface for debugging and/or monitoring the status of the unit from within the home network

*	Supported ArduinoOTA for code downloads over the network

*	mDNS for network address discovery

*	Parameters retained in the ESP32 using nonvolatile storage in the ESP32

*	A real-time clock, synchronized to an NTP time server, to turn off the unit during off-hours

*	Printed circuit board designed to fit within a standard plastic enclosure
