# A PWM fan controller using an ATtiny85

The code is written for an ATtiny 45/85. The *ATTinyCore* [https://github.com/SpenceKonde/ATTinyCore](https://github.com/SpenceKonde/ATTinyCore) is required for compiling.

The *SendOnlySoftwareSerial* library by Nick Gammon [https://github.com/nickgammon/SendOnlySoftwareSerial](https://github.com/nickgammon/SendOnlySoftwareSerial) is used for the serial output of the fan revolutions

```Text
Used Pins:
Pin 1 PB5 = Reset
Pin 2 PB3 = unsed
Pin 3 PB4 = ADC Input (Temperature Voltage)
Pin 4 GND
PIN 5 PB0 = mySerial Output
Pin 6 PB1 = PWM Signal Output
Pin 7 PB2 = PWM FAN Signal Input (Frequency Counter)
Pin 8 VCC = +5V
              +-\/-+
 NC     PB5 1 |    | 8 VCC   
 NC     PB3 2 |    | 7 PB2*  
       *PB4 3 |    | 6 PB1*  
        GND 4 |    | 5 PB0*  
              +----+ 
```

## An example circuit

[Circuit diagram]((/docu/PWM-FAN-Control.pdf?raw=true))
