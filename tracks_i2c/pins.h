#ifndef __PINS_H

#define __PINS_H

#define LEFT_PIN 9 //9
#define RIGHT_PIN 11 //11

#define HEAD_PWM_PIN 3 //3
#define BASE_TURN_PWM_PIN 5 //5
#define BASE_TILT_PWM_PIN 6 //6

#define LED_PIN LED_BUILTIN //13

#define BATTERY_PIN A0 //14
#define CHARGER_PIN A1 //15
#define CURRENT_SENSOR_PIN A2 //16

#define BATTERY_LED_PIN 11 //12
#define CURRENT_LED_PIN 12 //10
//temperature sensor

//I2C - A4,A5 (18,19)


//3 analog input, 4 pwm,  remaining, can use 4 more servos - 2 arm servos, 1 head scanner,
//after that 20 - 6 analog input = 14 - 2 serial = 12 - 5 pwm = 7 - 1 led = 6 (led for current, led for servo 1, led for servo 2, led for servo 3)

//2 motors, arm - 5 servo (2 bottom, 3 top)

//head scanner - 1 servo

//ioio - 2 wire - i2c, power (2 wire if it has own regulator)

//so arm - 3 servo, ioio, power regulator for arm servos so it is possible to use 2 power wire 7.5V for ioio and servo etc - total 4 wires

//divider 2 resistor for charger input, divider 2 resistor for battery input, current circuit


/*

//http://arduino.cc/playground/Learning/Pins

20 ping altogether

* Serial: 0 (RX) and 1 (TX). Used to receive (RX) and transmit (TX) TTL serial data. These pins are connected to the TX-0 and RX-1 pins of the six pin header.

* External Interrupts: 2 and 3. These pins can be configured to trigger an interrupt on a low value, a rising or falling edge, or a change in value. See the attachInterrupt() function for details.

* PWM: *3*, *5*, 6, *9*, *10*, and 11. Provide 8-bit PWM output with the analogWrite() function.

* SPI: 10 (SS), 11 (MOSI), *12* (MISO), 13 (SCK). These pins support SPI communication, which, although provided by the underlying hardware, is not currently included in the Arduino language.

* LED: *13*. There is a built-in LED connected to digital pin 13. When the pin is HIGH value, the LED is on, when the pin is LOW, it's off. 

*14*,*15*,16,17,*18*,*19* - 6 analog inputs, each of which provide 10 bits of resolution (i.e. 1024 different values). 
By default they measure from ground to VCC, though is it possible to change the upper end of their range using the AREF pin and some low-level code. 
Additionally, some pins have specialized functionality:

* I2C: 4 (SDA) and 5 (SCL). Support I2C (TWI) communication using the Wire library. (pins 18,19)

There are a couple of other pins on the board:

* AREF. Reference voltage for the analog inputs. Used with analogReference().

* Reset. Bring this line LOW to reset the microcontroller. Typically used to add a reset button to shields which block the one on the board. 


static const uint8_t SS = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK = 13;

static const uint8_t SDA = 18;
static const uint8_t SCL = 19;
static const uint8_t LED_BUILTIN = 13;

static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;


0 - rx
1 - tx
2
3 - head servo
4
5 - base turn servo
6 - base tilt servo
7
8
9 - left motor pin

10 - current led
11 - right motor pin
12 - battery led
13 - buildin led
14 - battery input
15 - charger input
16 - el. current input
17 
18 - SDA
19 - SCL


*/

#endif //__PINS_H
