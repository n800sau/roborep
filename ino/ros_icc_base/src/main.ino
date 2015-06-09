#include <voltage.h>
#include "../../include/printf.h"
#include <Wire.h>
#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include <Kalman.h>

// time in millisecs to stop if no encoder reading
#define TIME2STOP 10000
#define COUNT_PER_REV 20.0
#define WHEEL_DIAMETER 0.065
#define BASELINE 0.14

#define ENC_STEP (WHEEL_DIAMETER * M_PI / COUNT_PER_REV)
#define MAX_MOTOR_POWER 255
#define MIN_MOTOR_POWER 200

bool compass_mode = true;

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

int intentDir = 0;
int azimuth = -1;
int azimuth_allowance = 20;
int offset = 0;

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
	lCounter += (lReverse) ? -1 : 1;
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

void stop(bool reset_azimuth=false)
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

void setup()
{
	Serial.begin(115200);
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

void printState()
{
	Serial.print((compass_mode) ? "C mode" : "D mode");
	Serial.print(" DATA ");
	Serial.print("lsteps:");
	Serial.print(lDest);
	Serial.print(",rsteps:");
	Serial.print(rDest);
	Serial.print(",lpw:");
	Serial.print(lPower * ((lReverse) ? -1 : 1));
	Serial.print(",rpw:");
	Serial.print(rPower * ((rReverse) ? -1 : 1));
	Serial.print(",lcnt:");
	Serial.print(lCounter);
	Serial.print(",rcnt:");
	Serial.print(rCounter);
//	Serial.print(",x:");
//	Serial.print(x);
//	Serial.print(",y:");
//	Serial.print(y);
//	Serial.print(",fi:");
//	Serial.print(fi);
	Serial.print(",head:");
	Serial.print(headingDegrees);
	Serial.print(",dir:");
	Serial.print(intentDir);
	int intentOffset = -1;
	if(intentDir >= 0) {
		intentOffset = intentDir - headingDegrees;
		if(intentOffset > 180) intentOffset = 360 - intentOffset;
		else if(intentOffset < -180) intentOffset = 360 + intentOffset;
	}
	Serial.print(",off:");
	Serial.print(intentOffset);
	Serial.print(",azim:");
	Serial.print(azimuth);
	Serial.print(",off:");
	Serial.print(offset);
//	Serial.print(",acc_x:");
//	Serial.print(accel_scaled.XAxis);
//	Serial.print(",gyro_x:");
//	Serial.print((int)gyro.g.x);
	Serial.println("");
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

void loop()
{
	static int last_lCounter = -1;
	static int last_rCounter = -1;
	static unsigned long last_millis = 0;
	char val;
	process_compass();
	process_accel();
	process_gyro();
	unsigned long cur_millis = millis();
	unsigned long millisdiff = (last_millis > cur_millis) ? ((unsigned long) -1) - last_millis + cur_millis : cur_millis - last_millis;
//		Serial.print(compass_raw.XAxis);
//		Serial.print(":");
//		Serial.print(headingDegrees);
//		Serial.println("");
	if( millisdiff > TIME2STOP ) {
		stop(true);
	} else {
		if(last_lCounter != lCounter || last_rCounter != rCounter) {
			printState();
			last_lCounter=lCounter;
			last_rCounter=rCounter;
		}
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
	val = Serial.read();
	if(val!=-1)
	{
		Serial.println(val);
		switch(val)
		{
			case '`':
				stop(true);
				compass_mode = !compass_mode;
				break;
			case 'q':
				LstepSize ++;
				Serial.print("left step:");
				Serial.println(LstepSize);
				break;
			case 'z':
				LstepSize --;
				Serial.print("left step:");
				Serial.println(LstepSize);
				break;
			case 'e':
				RstepSize ++;
				Serial.print("right step:");
				Serial.println(RstepSize);
				break;
			case 'c':
				RstepSize --;
				Serial.print("right step:");
				Serial.println(RstepSize);
				break;
			case 'v':
				Serial.print("V:");
				Serial.println(readVccMv());
				break;
			case '0':
				Serial.println("Position reset");
				fi = x = y = 0;
				break;
			case 's'://stop
				stop(true);
				break;
		}
		if(compass_mode) {
			switch(val)
			{
				case 'w'://Move ahead
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = false;
					azimuth = -1;
					break;
				case 'x'://move back
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = true;
					azimuth = -1;
					break;
				case 'y': // go north
					intentDir = 0;
					break;
				case 'b': // go south
					intentDir = 180;
					break;
				case 'g': // go west
					intentDir = 360-90;
					break;
				case 'h': // go east
					intentDir = 90;
					break;
				case 'j': // reduce azimuth
					intentDir = (360 + intentDir - 10) % 360;
					break;
				case 'l': // enlarge azimuth
					intentDir = (360 + intentDir + 10) % 360;
					break;
				case 'k': // start direction
					azimuth = intentDir;
					break;
			}
		} else {
			switch(val)
			{
				case 'w'://Move ahead
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = false;
					break;
				case 'x'://move back
					stop();
					lDest = LstepSize;
					rDest = RstepSize;
					lReverse = rReverse = true;
					break;
				case 'a'://turn left
					stop();
					lDest = rDest = TstepSize;
					lReverse = true;
					rReverse = false;
					break;
				case 'd'://turn right
					stop();
					lDest = rDest = TstepSize;
					lReverse = false;
					rReverse = true;
					break;
			}
		}
		last_millis = cur_millis;
		printState();
	}
}
