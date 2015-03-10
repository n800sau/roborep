#include "motor_proc.h"
#include <HUBeeBMDWheel.h>

static HUBeeBMDWheel motor1Wheel;
static HUBeeBMDWheel motor2Wheel;

int motor1Speed = 100, motor2Speed = 100;

int motor1QeiAPin	 = 3; //external interrupt 1 (UNO) or 0 (Leonardo)
int motor1QeiBPin	 = 7;
int motor2QeiAPin = 2; //external interrupt 0 (UNO) or 1 (Leonardo)
int motor2QeiBPin = 4;

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
			break;
		case 1:
		case 2:
			motor1QeiCounts++;
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
			break;
		case 2:
			motor2QeiCounts--;
			break;
	}
	microTime = micros();
	motor2ElapsedTime = microTime-motor2OldElapsedTime;
	motor2OldElapsedTime = microTime;
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
}

void process_motors()
{
}

