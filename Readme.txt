Readme
******

Features
========
- PID controlled in all stages of operation for heating and cooling
- Servo-controlled door
- Cooling fan
- Multi-color LED indication for the varios states and conditions
- Buzzer indication for the various states and conditions
- Serial or parallel LCD support
- Ramped temperature control
- 16 x 2 LCD display
- Modular code allows easy removal of unused features

Required Libraries
==================
- LiquidTWI Library for high-performance i2c serial LCD
>> http://forums.adafruit.com/viewtopic.php?f=19&t=21586&p=113177

- Arduino PID Library: 
>> https://github.com/br3ttb/Arduino-PID-Library

- MAX31855 Library: 
>> https://github.com/rocketscream/MAX31855

IMPORTANT
=========
- Due to pin usage and additional feature support, this fork is not compatible with the RocketScream Reflow Oven Controller Shield.
- MAX6675 is no longer supported.
