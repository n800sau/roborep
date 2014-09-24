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
int rEN = 6;
int lEN = 5;
int rIN = 7;
int lIN = 4;


// encoder count before the left motor stops
volatile int lMotorDest = 0;
// left motor direction
volatile bool lMotorReverse = false;

// encoder count before the right motor stops
volatile int rMotorDest = 0;
// right motor direction
volatile bool rMotorReverse = false;

// left motor default power
int lPower = 255;
// right motor default power
int rPower = 230;

// encoder pins
int Eleft = 3;
int Eright = 2;

// encoder interrupts
int lInt = Eleft - 2;
int rInt = Eright - 2;

// calculated coordinates and angle
volatile float angl = 0, x = 0, y = 0;

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


// left encoder steps total
volatile long lEncSteps = 0;
// left encoder steps total of the last calculation
long lMSteps = 0;

// right encoder steps total
volatile long rEncSteps = 0;
// right encoder steps total of the last calculation
long rMSteps = 0;

// last calculation time
volatile unsigned long last_millis = millis();

// Working variables for the interrupt routines
//
volatile unsigned long intLtime = 0;
volatile unsigned long intRtime = 0;
volatile uint8_t intLsignal = 0;
volatile uint8_t intRsignal = 0;
volatile uint8_t intLhistory = 0;
volatile uint8_t intRhistory = 0;

void resetLastMillis()
{
	last_millis = millis();
}

void lIntCB()
{
	if( micros() - intLtime < threshold )
		return;
	resetLastMillis();
	intLhistory = intLsignal;
	intLsignal = bitRead(PIND,Eright);
	if ( intLhistory==intLsignal )
		return;
	intLtime = micros();
	if(lMotorDest) {
		lMotorDest += (lMotorDest < 0) ? 1 : -1;
	}
	int v = (lMotorReverse) ? -1 : 1;
	lEncSteps += v;
}

void rIntCB()
{
	if (micros() - intRtime < threshold )
		return;
	resetLastMillis();
	intRhistory = intRsignal;
	intRsignal = bitRead(PIND,Eleft);
	if ( intRhistory==intRsignal )
		return;
	intRtime = micros();
	if(rMotorDest) {
		rMotorDest += (rMotorDest < 0) ? 1 : -1;
	}
	int v = (rMotorReverse) ? -1 : 1;
	rEncSteps += v;
}


void setLeftMotor(int pwm, int steps)
{
	resetLastMillis();
	if(lMotorDest && steps == 0) {
		Serial.println("Stop Left");
	}
	if(steps) {
		lMotorDest += steps;
	} else {
		lMotorDest = 0;
	}
	// if motor stopped the direction should not be changed
	// to prevent encoder to count back while the wheel is stopping
	if(lMotorDest) {
		lMotorReverse = lMotorDest < 0;
		analogWrite(lEN, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
		digitalWrite(lIN, (lMotorReverse) ? LOW : HIGH);
	} else {
		analogWrite(lEN, 0);
	}
}

void setRightMotor(int pwm, int steps)
{
	resetLastMillis();
	if(rMotorDest && steps == 0) {
		Serial.println("Stop Right");
	}
	if(steps) {
		rMotorDest += steps;
	} else {
		rMotorDest = 0;
	}
	// if motor stopped the direction should not be changed
	// to prevent encoder to count back while the wheel is stopping
	if(rMotorDest) {
		rMotorReverse = rMotorDest < 0;
		analogWrite(rEN, pwm);
		digitalWrite(rIN, (rMotorReverse) ? LOW : HIGH);
	} else {
		analogWrite(rEN, 0);
	}
}

void stop()
{
	setLeftMotor(0, 0);
	setRightMotor(0, 0);
}

int lastLVal = -1;
int lastRVal = -1;

void printDests()
{
	Serial.print("M ");
	Serial.print(lMotorDest);
	Serial.print(" ");
	Serial.println(rMotorDest);
}

void printEncoders()
{
	int lval = lEncSteps;
	int rval = rEncSteps;
	if(lastLVal != lval || lastRVal != rval) {
		Serial.print(lastLVal=lval);
		Serial.print(" ");
		Serial.print(lastRVal=rval);
		Serial.print(" ");
		Serial.print(x);
		Serial.print(" ");
		Serial.println(y);
	}
}

float xypos(int left, int right)
{
	if(left || right) {
		Serial.print("left=");
		Serial.print(left);
		Serial.print(", right=");
		Serial.println(right);
	}
	float Sl = ENC_STEP * left;
	float Sr = ENC_STEP * right;
	float s = (Sr + Sl) / 2;
	angl = (Sr - Sl) / BASELINE + angl;
	x = s * cos(angl) + x;
	y = s * sin(angl) + y;
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

void keep_straight(bool reverse)
{
	int diff = abs(rMotorDest - lMotorDest);
	if(diff > 1) {
		if(reverse) {
			if(rMotorDest > lMotorDest) {
				// speed up right
			} else {
				// speed up left
			}
		} else {
			if(rMotorDest > lMotorDest) {
				// speed up left
			} else {
				// speed up right
			}
		}
		printf("DIFF=%d\r\n", diff);
	}
}

void loop()
{
	char val;
	while(1)
	{
		if(lMotorReverse == rMotorReverse && (lMotorDest || lMotorDest) ) {
			keep_straight(lMotorReverse);
		}
		printEncoders();
		xypos(lEncSteps - lMSteps, rEncSteps - rMSteps);
		lMSteps = lEncSteps;
		rMSteps = rEncSteps;
		if( ((lMotorReverse) ? -lMotorDest : lMotorDest ) <= 0) {
			setLeftMotor(0, 0);
		}
		if( ((rMotorReverse) ? -rMotorDest : rMotorDest ) <= 0) {
			setRightMotor(0, 0);
		}
		unsigned long cur_millis = millis();
		unsigned long millisdiff = (last_millis > cur_millis) ? ((unsigned long) -1) - last_millis + cur_millis : cur_millis - last_millis;
		if( millisdiff > TIME2STOP) {
			stop();
		}
		val = Serial.read();
		if(val!=-1)
		{
			Serial.println(val);
			switch(val)
			{
				case 'w'://Move ahead
					stop();
					setLeftMotor(lPower,stepSize);
					setRightMotor(rPower,stepSize);
					break;
				case 'x'://move back
					stop();
					setLeftMotor(lPower,-stepSize);
					setRightMotor(rPower,-stepSize);
					break;
				case 'a'://turn left
					stop();
					setLeftMotor(lPower,-stepSize);
					setRightMotor(rPower,stepSize);
					break;
				case 'd'://turn right
					stop();
					setLeftMotor(lPower,stepSize);
					setRightMotor(rPower,-stepSize);
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
		}
	}
}
