# Introduction
This is the software for the Raspberry Pi controlling the table tennis ball fetcher robot called
ASH.

It's basically divided into these categories:
- Computer vision;
- Peripherals data reading;
- Peripherals actuation;

# GPIO configuration:
Here is the GPIO configuration for Raspberry Pi boards:
![GPIO pins in Raspberry Pi](images/image.png)

## Used pins
GPIO12 (Pin 33 - PWM0) - Right Motor EN
GPIO16 (Pin 36) - Right Motor IN1
GPIO20 (Pin 38) - Right Motor IN2
GPIO21 (Pin 40) - Right Motor encoder feedback 1
GPIO25 (Pin 22) - Right Motor encoder feedback 2

GPIO13 (Pin 33 - PWM1) - Left Motor EN
GPIO19 (Pin 35) - Left Motor IN1
GPIO26 (Pin 37) - Left Motor IN2
GPIO6  (Pin 31) - Left Motor encoder feedback 1
GPIO5  (Pin 29) - Left Motor encoder feedback 2

GPIO24 (Pin 18) - Servo Motor PWM

GPIO2  (Pin 3) - I2C SDA
GPIO3  (Pin 5) - I2C SCL

GPIO23 (Pin 16) - Hall effect sensor

GPIO14 (Pin 8) - Infrared sensor front
GPIO15 (Pin 10) - Infrared sensor back

GPIO17 (Pin 11) - Ultrasonic sensor trigger FOR ALL 4 SENSORS
GPIO27 (Pin 13) - Ultrasonic sensor 1 echo
GPIO22 (Pin 15) - Ultrasonic sensor 2 echo
GPIO10 (Pin 19) - Ultrasonic sensor 3 echo
GPIO9  (Pin 21) - Ultrasonic sensor 4 echo

## Available pins
GPIO4  (Pin 7)
GPIO7  (Pin 26)
GPIO8  (Pin 24)
GPIO11 (Pin 23)
GPIO18 (Pin 12)
