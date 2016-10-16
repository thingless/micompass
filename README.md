micompass
---------
A minecraft compass for Arduino!

Building
--------
1. Link this directory to ~/Arduino
2. Open the Arduino software
3. Select Arduino Nano & upload

Hardware
--------
* Arduino Nano clone from eBay [link](http://www.ebay.com/itm/262123424219)
* GY-271 compass module
* GY-GPS6MV2 GPS module

Pinouts
-------
* Pin D6  - TX on the GPS
* Pin D7  - RX on the GPS
* Pin A4  - SDA on the compass (also SDA on the Nano)
* Pin A5  - SCL on the compass (also SCL on the Nano)
* Pin D10 - Button to set waypoint (normally low, with external pull-down)
* Pin D13 is the built-in LED

Notes
-----
Compass module: X is north, uses I2C, by default Z must be up to work

Todos
-----
* Figure out better blink pattern
* Calibrate compass & handle non-horizontal modes
* Play with LCD (need level shifters)
* External power supply
* Switch to a Teensy?
* Try internal pull-down for button?
