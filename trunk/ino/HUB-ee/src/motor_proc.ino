#include "motor_proc.h"
#include <HUBeeBMDWheel.h>

static HUBeeBMDWheel motor1Wheel;
static HUBeeBMDWheel motor2Wheel;

static const int min_pwr = 30;
static const int max_pwr = 100;

static int motor1Speed = 100, motor2Speed = 100;

float motor1Coef = 1, motor2Coef = 1;

int motor1QeiAPin	 = 3; //external interrupt 1 (UNO) or 0 (Leonardo)
int motor1QeiBPin	 = 7;
int motor2QeiAPin = 2; //external interrupt 0 (UNO) or 1 (Leonardo)
int motor2QeiBPin = 4;

// counts to move
volatile int lCount = 0, rCount = 0;
// to which direction (1, -1)
volatile int lDir = 0, rDir = 0;

volatile unsigned long stopTime = millis();

volatile int motor1QeiCounts = 0, motor2QeiCounts = 0;
volatile int oldMotor1QeiCounts = 0, oldMmotor2QeiCounts = 0;
volatile unsigned long int motor1ElapsedTime = 0, motor2ElapsedTime = 0;
volatile unsigned long int motor1OldElapsedTime = 0, motor2OldElapsedTime = 0;

void Motor1quickQEI()
{
	//a fast(ish) QEI function
	int state = 0;
	unsigned long int microTime;
	oldMotor1QeiCounts = motor1QeiCounts;
	state = digitalRead(motor1QeiAPin) << 1;
	state = state|digitalRead(motor1QeiBPin);
	switch (state)
	{
		case 0:
		case 3:
			motor1QeiCounts--;
			if(lDir < 0 && lCount > 0) lCount--;
			break;
		case 1:
		case 2:
			motor1QeiCounts++;
			if(lDir > 0 && lCount > 0) lCount--;
			break;
	}

	microTime = micros();
	motor1ElapsedTime = microTime-motor1OldElapsedTime;
	motor1OldElapsedTime = microTime;
}

void Motor2quickQEI()
{
	//a fast(ish) QEI function
	int state = 0;
	int microTime;
	state = digitalRead(motor2QeiAPin) << 1;
	state = state|digitalRead(motor2QeiBPin);
	switch (state)
	{
		case 0:
		case 3:
			motor2QeiCounts++;
			if(rDir > 0 && rCount > 0) rCount--;
			break;
		case 2:
			motor2QeiCounts--;
			if(rDir < 0 && rCount > 0) rCount--;
			break;
	}
	microTime = micros();
	motor2ElapsedTime = microTime-motor2OldElapsedTime;
	motor2OldElapsedTime = microTime;
}

void stop()
{
	lCount = 0;
	rCount = 0;
	lDir = 0;
	rDir = 0;
	motor1Wheel.stopMotor();
	motor2Wheel.stopMotor();
}

void mv_forward(int ms)
{
	lCount = 10;
	rCount = 10;
	lDir = 1;
	rDir = 1;
	stopTime = millis() + ms;
}

void mv_back(int ms)
{
	lCount = 10;
	rCount = 10;
	lDir = -1;
	rDir = -1;
	stopTime = millis() + ms;
}

void turn_left(int ms)
{
	lCount = 10;
	rCount = 10;
	lDir = -1;
	rDir = 1;
	stopTime = millis() + ms;
}

void turn_right(int ms)
{
	lCount = 10;
	rCount = 10;
	lDir = 1;
	rDir = -1;
	stopTime = millis() + ms;
}

void calibrate_motors()
{
	int m1Q = motor1QeiCounts;
	int m2Q = motor2QeiCounts;
	float c1, c2;
	stop();
	delay(1000);
	motor1QeiCounts = motor2QeiCounts = 0;
	motor1Coef = motor2Coef = 1;
	motor1Wheel.setMotorPower(motor1Speed);
	motor2Wheel.setMotorPower(motor2Speed);
	delay(1000);
	stop();
	if(motor2QeiCounts && motor1QeiCounts) {
		c1 = sqrt(float(motor2QeiCounts) / motor1QeiCounts);
		c2 = 1 / c1;
		delay(1000);
		motor1Wheel.setMotorPower(-motor1Speed);
		motor2Wheel.setMotorPower(-motor2Speed);
		delay(1000);
		stop();
		motor1QeiCounts = m1Q;
		motor2QeiCounts = m2Q;
		motor1Coef = c1;
		motor2Coef = c2;
	} else {
		motor1Wheel.setMotorPower(-motor1Speed);
		motor2Wheel.setMotorPower(-motor2Speed);
		delay(1000);
		stop();
	}
}

void setup_motors()
{
	pinMode(motor1QeiAPin, INPUT_PULLUP);
	pinMode(motor2QeiAPin, INPUT_PULLUP);
	pinMode(motor1QeiBPin, INPUT_PULLUP);
	pinMode(motor2QeiBPin, INPUT_PULLUP);
	motor1Wheel.setupPins(11,8,9); //setup using pins 12 and 2 for direction control, and 3 for PWM speed control
	motor2Wheel.setupPins(12,13,10);//setup using pins 13 and 4 for direction control, and 11 for PWM speed control
	motor1Wheel.setDirectionMode(1); //Set the direction mode to 1
	motor2Wheel.setDirectionMode(1); //set the direction mode to 1
	motor1Wheel.setBrakeMode(true);
	motor2Wheel.setBrakeMode(true);
	/*
	The counter for each wheel (motor1QeiCounts and motor2QeiCounts) will go up when the motor direction is set to 1 and motor power is positive
	It will go down when motor power is negative
	The motor1ElapsedTime and motor2ElapsedTime will give you an indication of speed - The bigger the number the lower the speed
	BE WARNED - The speed variables will give innacurate readings at low speed because the values get too big
	*/
	
	
	attachInterrupt(1, Motor1quickQEI, CHANGE);
	attachInterrupt(0, Motor2quickQEI, CHANGE);
	//start the wheels
	motor1ElapsedTime = micros();
	motor2ElapsedTime = micros();

//		motor1Wheel.setMotorPower(200); //full speed ahead
//		motor2Wheel.setMotorPower(200); //full speed ahead
//		delay(1000);
//		motor1Wheel.stopMotor();
//		motor2Wheel.stopMotor();
}

void process_motors()
{
	int pwr;
	if(stopTime > millis()) {
		if(lCount > 0) {
			pwr = lDir * motor1Speed * motor1Coef;
			if(lCount < 5) {
				pwr -= (pwr - min_pwr) / 5. * (5 - lCount);
			}
			motor1Wheel.setMotorPower(pwr);
		} else {
			lDir = 0;
			motor1Wheel.stopMotor();
		}
		if(rCount > 0) {
			pwr = rDir * motor2Speed * motor2Coef;
			if(rCount < 5) {
				pwr -= (pwr - min_pwr) / 5. * (5 - rCount);
			}
			motor2Wheel.setMotorPower(rDir * motor2Speed * motor2Coef);
		} else {
			rDir = 0;
			motor2Wheel.stopMotor();
		}
	} else {
		stop();
	}
}

