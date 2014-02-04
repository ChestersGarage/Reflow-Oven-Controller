Readme
******

!!! Currently incomplete and untested !!!

Goals (See ChangeLog.txt for implementation status):
- Add LiquidTWI library for Adafruit i2c LCD Backpack
- Servo-controlled door
- Cooling fan
- PID for the cool stage
 
Additional Required Libraries
=============================
- LiquidTWI Library for high-performance i2c serial LCD
>> http://forums.adafruit.com/viewtopic.php?f=19&t=21586&p=113177

IMPORTANT:
- Sorry, this fork is not compatible with the RocketScream Reflow Oven Controller Shield.
- If you use the MAX6675 at the same time as the LiquidTWI library and Adafruit i2c LCD Backpack, you must choose different pins for the MAX6675 to use for SPI connection, because the LCD backpack needs hardware i2c, which requires pins A4 and A5. Fortunately, lots of pins become available when using the LCD backpack.

