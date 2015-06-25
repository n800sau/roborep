#include <Kalman.h>
#include "hmc5883l_proc.h"
#include "motor_proc.h"


#define COUNT_PER_REV 20.0
#define WHEEL_DIAMETER 0.065
#define BASELINE 0.14

#define ENC_STEP (WHEEL_DIAMETER * M_PI / COUNT_PER_REV)
#define MAX_MOTOR_POWER 255
#define MIN_MOTOR_POWER 200

// count of encoder ticks until stop
int LstepSize = 5;
int RstepSize = 5;

// turn step size
int TstepSize = 4;

// motor pins
const int rEN = 6;
const int lEN = 5;
const int rIN = 7;
const int lIN = 4;

// movement

bool compass_mode = false;


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

// encoder distance counter
volatile int lCounter = 0;
volatile int rCounter = 0;

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

int intentDir = 0;
int azimuth = -1;
int azimuth_allowance = 20;
int offset = 0;


void lIntCB()
{
	if( micros() - intLtime < threshold )
		return;
	intLhistory = intLsignal;
	intLsignal = bitRead(PIND,Eright);
	if ( intLhistory==intLsignal )
		return;
	intLtime = micros();
	lCounter += (lReverse) ? -1 : 1;
	if(azimuth < 0) {
		lDest--;
	}
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
	rCounter += (rReverse) ? -1 : 1;
	if(azimuth < 0) {
		rDest--;
	}
	calc_xy(0, (rReverse) ? -1 : 1);
}

void setLeftMotor(int pwm)
{
	// if motor stopped the direction should not be changed
	// to prevent encoder to count back while the wheel is stopping
	if(lPower != pwm && pwm == 0) {
		printf("Left Stop, Right: %d\r\n", rPower);
	}
//	printf("Left power: %d\n\r", pwm);
	analogWrite(lEN, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
	lPower = pwm;

	int in = (lReverse) ? LOW : HIGH;
//	int en = (pwm == 0) ? LOW : HIGH;
//	printf("Left: %d:%d\n\r", in, en);

	digitalWrite(lIN, in);

	// instead of analogWrite
//	digitalWrite(lEN, en);
}

void setRightMotor(int pwm)
{
	// if motor stopped the direction should not be changed
	// to prevent encoder to count back while the wheel is stopping
	if(rPower != pwm && pwm == 0) {
		printf("Left: %d, Right Stop\r\n", lPower);
	}
//	printf("Right power: %d\n\r", pwm);
	analogWrite(rEN, pwm);
	rPower = pwm;

	int in = (rReverse) ? LOW : HIGH;
//	int en = (pwm == 0) ? LOW : HIGH;
//	printf("Right: %d:%d\n\r", in, en);

	digitalWrite(rIN, in);

	// instead of analogWrite
//	digitalWrite(rEN, en);
}

void stop(bool reset_azimuth)
{
	setLeftMotor(0);
	setRightMotor(0);
	lDest = rDest = 0;
	if(reset_azimuth) {
		azimuth = -1;
	}
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

void updateMove()
{
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

void mv_forward(int steps)
{
	stop();
	lDest = LstepSize * steps;
	rDest = RstepSize * steps;
	lReverse = rReverse = false;
}

void mv_back(int steps)
{
	stop();
	lDest = LstepSize * steps;
	rDest = RstepSize * steps;
	lReverse = rReverse = true;
}

void turn_left(int steps)
{
	stop();
	lDest = rDest = TstepSize * steps;
	lReverse = true;
	rReverse = false;
}

void turn_right(int steps)
{
	stop();
	lDest = rDest = TstepSize * steps;
	lReverse = false;
	rReverse = true;
}


void motor_setup()
{
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

void motor_process()
{
	if(compass_mode) {
		if(azimuth >= 0) {
			offset = azimuth - headingDegrees;
			if(offset > 180) offset = 360 - offset;
			else if(offset < -180) offset = 360 + offset;
			int pwm = 210 + 55 * abs(offset) / 180;
			if( offset > azimuth_allowance) {
				lReverse = false;
				rReverse = true;
				setLeftMotor(pwm);
				setRightMotor(pwm);
			} else if ( offset < -azimuth_allowance ) {
				lReverse = true;
				rReverse = false;
				setLeftMotor(pwm);
				setRightMotor(pwm);
			} else {
				stop();
			}
		} else {
			updateMove();
		}
	} else {
		updateMove();
	}
}

