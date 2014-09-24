#include <voltage.h>
#include "../../include/printf.h"

// time in millisecs to stop if no encoder reading
#define TIME2STOP 10000
#define COUNT_PER_REV 20.0
#define WHEEL_DIAMETER 0.065
#define BASELINE 0.14

#define ENC_STEP (COUNT_PER_REV * WHEEL_DIAMETER / M_PI)

// count of encoder ticks until stop
int stepSize = 5;

// motor pins
const int rEN = 6;
const int lEN = 5;
const int rIN = 7;
const int lIN = 4;

// movement

// left reverse
volatile bool lReverse = false;
// right reverse
volatile bool rReverse = false;


// encoder pins
const int Eleft = 2;
const int Eright = 3;

// encoder interrupts
const int lInt = Eleft - 2;
const int rInt = Eright - 2;

volatile int lDest = 0;
volatile int lPower = 0;

volatile int rDest = 0;
volatile int rPower = 0;

// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder. 
//
// The threshold value I'm using limits it to 100 half pulses a second
//
// My encoder has 12 pulses per 360deg rotation and the specs say
// it is rated at a maximum of 100rpm.
//
// This threshold will permit my encoder to reach 250rpm so if it was connected
// to a motor instead of a manually operated knob I
// might possibly need to adjust it to 25000. However, this threshold
// value is working perfectly for my situation
//
volatile unsigned long threshold = 10000;

// Working variables for the interrupt routines
//
volatile unsigned long intLtime = 0;
volatile unsigned long intRtime = 0;
volatile uint8_t intLsignal = 0;
volatile uint8_t intRsignal = 0;
volatile uint8_t intLhistory = 0;
volatile uint8_t intRhistory = 0;

void lIntCB()
{
	if( micros() - intLtime < threshold )
		return;
	intLhistory = intLsignal;
	intLsignal = bitRead(PIND,Eright);
	if ( intLhistory==intLsignal )
		return;
	intLtime = micros();
	lDest--;
}

void rIntCB()
{
	if (micros() - intRtime < threshold )
		return;
	intRhistory = intRsignal;
	intRsignal = bitRead(PIND,Eleft);
	if ( intRhistory==intRsignal )
		return;
	intRtime = micros();
	rDest--;
}

void setLeftMotor(int pwm)
{
	// if motor stopped the direction should not be changed
	// to prevent encoder to count back while the wheel is stopping
	if(lPower != pwm && pwm == 0) {
		printf("Left Stop, Right: %d\r\n", rPower);
	}
	analogWrite(lEN, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
	lPower = pwm;
	digitalWrite(lIN, (lReverse) ? LOW : HIGH);
}

void setRightMotor(int pwm)
{
	// if motor stopped the direction should not be changed
	// to prevent encoder to count back while the wheel is stopping
	if(rPower != pwm && pwm == 0) {
		printf("Left: %d, Right Stop\r\n", lPower);
	}
	analogWrite(rEN, pwm);
	rPower = pwm;
	digitalWrite(rIN, (rReverse) ? LOW : HIGH);
}

void stop()
{
	setLeftMotor(0);
	setRightMotor(0);
	lDest = rDest = 0;
}

void setup()
{
	Serial.begin(57600);
	printf_begin();
	pinMode(lEN, OUTPUT);
	pinMode(rEN, OUTPUT);
	pinMode(lIN, OUTPUT);
	pinMode(rIN, OUTPUT);
	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);
	digitalWrite(Eleft, HIGH);
	digitalWrite(Eright, HIGH);
	attachInterrupt(lInt, rIntCB, CHANGE);
	attachInterrupt(rInt, lIntCB, CHANGE);
}

void printEncoders()
{
	static int last_lDest = -1;
	static int last_rDest = -1;
	if(last_lDest != lDest || last_rDest != rDest) {
		Serial.print(last_lDest=lDest);
		Serial.print(" ");
		Serial.println(last_rDest=rDest);
//		Serial.print(" ");
//		Serial.print(x);
//		Serial.print(" ");
//		Serial.println(y);
	}
}

void loop()
{
	static unsigned long last_millis = 0;
	char val;
	while(1)
	{
		unsigned long cur_millis = millis();
		unsigned long millisdiff = (last_millis > cur_millis) ? ((unsigned long) -1) - last_millis + cur_millis : cur_millis - last_millis;
		if( millisdiff > TIME2STOP ) {
			stop();
		} else {
			printEncoders();
			if(lDest <= 0 && rDest <= 0) {
				stop();
			} else {
				if(lDest > 0) {
//					setLeftMotor(250);
					setLeftMotor(200 + ((lDest > 2) ? 55 : 10));
				} else {
					setLeftMotor(0);
				}
				if(rDest > 0) {
//					setRightMotor(250);
					setRightMotor(200 + ((rDest > 2) ? 55 : 10));
				} else {
					setRightMotor(0);
				}
			}
		}
		val = Serial.read();
		if(val!=-1)
		{
			Serial.println(val);
			switch(val)
			{
				case 'w'://Move ahead
					stop();
					lDest = rDest = stepSize;
					lReverse = rReverse = false;
					break;
				case 'x'://move back
					stop();
					lDest = rDest = stepSize;
					lReverse = rReverse = true;
					break;
				case 'a'://turn left
					stop();
					lDest = rDest = stepSize;
//					lDest = 0;
//					rDest = stepSize;
					lReverse = false;
					rReverse = true;
					break;
				case 'd'://turn right
					stop();
					lDest = rDest = stepSize;
//					lDest = stepSize;
//					rDest = 0;
					lReverse = true;
					rReverse = false;
					break;
				case 's'://stop
					stop();
					break;
				case 'p':
					stepSize ++;
					Serial.println(stepSize);
					break;
				case 'l':
					stepSize --;
					Serial.println(stepSize);
					break;
				case 'v':
					Serial.print("V:");
					Serial.println(readVccMv());
					break;
			}
			last_millis = cur_millis;
		}
	}
}
