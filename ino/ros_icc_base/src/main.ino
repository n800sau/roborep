#include <voltage.h>
#include "../../include/printf.h"
#include <Wire.h>
#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"

// time in millisecs to stop if no encoder reading
#define TIME2STOP 10000
#define COUNT_PER_REV 20.0
#define WHEEL_DIAMETER 0.065
#define BASELINE 0.14

#define ENC_STEP (WHEEL_DIAMETER * M_PI / COUNT_PER_REV)
#define MAX_MOTOR_POWER 255
#define MIN_MOTOR_POWER 200

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
	calc_xy((lReverse) ? -1 : 1, 0);
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
	calc_xy(0, (rReverse) ? -1 : 1);
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

volatile double fi = 0, x = 0, y = 0;

void calc_xy(double l, double r)
{
	l *= ENC_STEP;
	r *= ENC_STEP;
	double dist = (l+r)/2.;
	fi = (r - l) / BASELINE + fi;
	x = dist * cos(fi) + x;
	y = dist * sin(fi) + y;
}

void setup()
{
	Serial.begin(57600);
	printf_begin();

	Serial.println("Starting the I2C interface.");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();

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
		Serial.print("DATA ");
		Serial.print("left:");
		Serial.print(last_lDest=lDest);
		Serial.print(",right:");
		Serial.print(last_rDest=rDest);
		Serial.print(",x:");
		Serial.print(x);
		Serial.print(",y:");
		Serial.print(y);
		Serial.print(",fi:");
		Serial.print(fi);
		Serial.print(",head:");
		Serial.print(headingDegrees);
		Serial.print(",acc_x:");
		Serial.print(accel_scaled.XAxis);
		Serial.print(",gyro_x:");
		Serial.println((int)gyro.g.x);
	}
}

void loop()
{
	static unsigned long last_millis = 0;
	char val;
	process_compass();
	process_accel();
	process_gyro();
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
				setLeftMotor((lDest > 2) ? MAX_MOTOR_POWER : MIN_MOTOR_POWER);
			} else if(lDest < 0) {
				Serial.println("Left reverse correction");
				lReverse = !lReverse;
				lDest = -lDest;
			} else {
				setLeftMotor(0);
			}
			if(rDest > 0) {
				setRightMotor((rDest > 2) ? MAX_MOTOR_POWER : MIN_MOTOR_POWER);
			} else if(rDest < 0) {
				Serial.println("Right reverse correction");
				rReverse = !rReverse;
				rDest = -rDest;
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
				lReverse = true;
				rReverse = false;
				break;
			case 'd'://turn right
				stop();
				lDest = rDest = stepSize;
				lReverse = false;
				rReverse = true;
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
			case 'z':
				Serial.println("Position reset");
				fi = x = y = 0;
				break;
		}
		last_millis = cur_millis;
	}
}
