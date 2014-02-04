Readme
******

Goals:
- Added LiquidTWI library for Adafruit 12c LCD Backpack
- Servo-controlled door
- Cooling fan
- PID for the cool stage
 
Additional Required Libraries
=============================
- LiquidTWI Library for high-performance 12c serial LCD
>> http://forums.adafruit.com/viewtopic.php?f=19&t=21586&p=113177
 
IMPORTANT!: If you use the MAX6675 and the LiquidTWI/Adafruit i2c LCD
Backpack at the same time, you must choose different pins for the 
MAX6675 to use for SPI connection because the LCD backpack needs hardware
i2c, which requires pins A4 and A5. Fortunately lots of pins become available
when using the LCD backpack.
