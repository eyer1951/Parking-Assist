# Web Application

For all of the different ESP32 projects I have built so far, it has been helpful to provide a user interface whereby various operating parameters could be set, 
and adjusted over time as needed. A general approach was developed, involving:

*	A browser-based user interface written in (of course) Javascript and HTML/CSS.

*	Use of Websockets to communicate between the project and the browser.

*	The ESP32 discovers and connects to the home network in station mode, sets up an mDNS identifier for TCP/IP, and then opens a WebSocket interface.

*	Parameters are represented by an array of up to 30 unsigned 32-bit integers.

*	The web application uses the mDNS ID to establish a WebSocket connection, then queries the project to retrieve the values of all parameters.

*	The ESP32 saves all parameters in nonvolatile storage (NVS).

*	Changes to any parameter are delivered immediately via the Websocket interface and saved in the ESP32 in NVS.

*	The set of parameters and definitions for the project are defined in a JSON document opened by the web application.

Note: because it loads a local JSON file, the web application must be served from a web server on your network rather than being executed as a file on 
the local computer (that is, unless you run the browser with “same origin policy” disabled). 
Otherwise you might see this kind of error in the JavaScript console: 

```Access to XMLHttpRequest at 'file:///C:/Users/ … /projectName.json' from origin 'null' has been blocked by CORS policy: Cross origin requests are only supported for protocol schemes: http, data, chrome, chrome-extension, chrome-untrusted, https.```

The JSON document, in addition to defining the name of the project (for the web page title), version number, and mDNS name, 
defines a number of operating parameters. Operating parameters can be values representable by range sliders, Boolean flags 
(represented by checkboxes), or colors established by “color picker” interface. The web interface for the present project did 
not utilize any checkboxes or color pickers and is shown here:

![](images/web-app.jpg)

The checkmark in the upper left indicates connectivity to the project. The “ERASE” button in the upper left is used to command the project to erase NVS, if needed.
The JSON document defines a number of characteristics of each range slider, including:

*	The name (label on left side)

* The minimum and maximum values of the slider.

*	The step size 

As an example of the latter, the time of day parameters are represented as a number of minutes since midnight, so a parameter that would represent midnight to noon would be specified with a range that goes from zero to 12 * 60 = 360. The name of that display function is “timeOfDay.” Other example display functions are:

*	percent – displays the slider as a whole percentage, where zero is all the way to the left and 100 is all the way to the right.

*	percentTenths – displayed as percent as above, but as a floating point number with one digit to the right of the decimal

*	tenths – displays the value of the slider, but divided by 10.

Other functions can be easily written as needed.

CSS can be included in the JSON definition of any range slider, to provide, for example, a horizontal bar or extra space associated with a given slider.
A link to the JSON document that produced the user interface above is https://github.com/eyer1951/Parking-Assist/blob/main/WebApp/pc/parkme.json.
