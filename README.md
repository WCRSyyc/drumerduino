# drummerduino

## Arduino Drummer

Arduino sketch to control drum sticks to beat a physical drum.

This is currently a shell sketch to get a simple structure that will allow adequate
control of a couple of drum sticks attached to and driven by a couple of electric
car door lock mechanisms.  The motors are driven through an Adafruit motor shield
(version 2), using their library.

Future idea: move the main control logic to a library, then just have the main
sketch feed the beat timings to the instance(s).  Variations: single drumming
instance control both sticks, or a separate instance for each stick.
