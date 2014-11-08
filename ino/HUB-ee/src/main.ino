#include <HUBeeBMDWheel.h>
#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"

int MVpin = A2;
int IRpin = A3;

HUBeeBMDWheel motor1Wheel;
HUBeeBMDWheel motor2Wheel;

int motor1Speed = 100, motor2Speed = 100;

int motor1QeiAPin	 = 3; //external interrupt 1 (UNO) or 0 (Leonardo)
int motor1QeiBPin	 = 7;
int motor2QeiAPin = 2; //external interrupt 0 (UNO) or 1 (Leonardo)
int motor2QeiBPin = 4;

volatile int motor1QeiCounts = 0, motor2QeiCounts = 0;
volatile int oldMotor1QeiCounts = 0, oldMmotor2QeiCounts = 0;
volatile unsigned long int motor1ElapsedTime = 0, motor2ElapsedTime = 0;
volatile unsigned long int motor1OldElapsedTime = 0, motor2OldElapsedTime = 0;

int serialTimer = 0;
int directionTimer = 1000;
int samples;
unsigned long int sampleSum = 0;
unsigned long int speedSaturation;
int count = 0;
bool lastMVfound = false;

void setup()
{
	Serial.println("Starting the I2C interface.");
	Wire.begin(); // Start the I2C interface.

	setup_compass();
	setup_accel();
	setup_gyro();

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
	Serial.begin(57600);
	//start the wheels
	motor1ElapsedTime = micros();
	motor2ElapsedTime = micros();
	sampleSum = 0;
	Serial.println("STARTING");
	speedSaturation = 32767;
}

void printState()
{
	Serial.print("head:");
	Serial.print(headingDegrees);
//	Serial.print(",acc_x:");
//	Serial.print(accel_scaled.XAxis);
//	Serial.print(",gyro_x:");
//	Serial.print((int)gyro.g.x);
	Serial.println("");
}

void loop()
{
	process_compass();
	process_accel();
	process_gyro();
	printState();
	bool MVfound = analogRead(MVpin) > 300;
	if(MVfound != lastMVfound) {
		lastMVfound = MVfound;
		if(MVfound) {
			count = 1;
		}
	}
	float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
	float distance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)
//	Serial.print("Distance ");
//	Serial.print(distance);
//	Serial.print(" Sensor ");
//	Serial.println(analogRead(MVpin));
	if(distance < 50) {
		motor1Wheel.setMotorPower(-motor1Speed); //full speed ahead
		motor2Wheel.setMotorPower(-motor2Speed); //full speed ahead
		delay(1000);
	} else if(count > 0) {
		count--;
		Serial.print(" Count 1: ");
		Serial.print(motor1QeiCounts);
		Serial.print(" Count 2: ");
		Serial.println(motor2QeiCounts);
		motor1Wheel.setMotorPower(motor1Speed); //full speed ahead
		motor2Wheel.setMotorPower(motor2Speed); //full speed ahead
		delay(1000);
		motor1Wheel.stopMotor();
		motor2Wheel.stopMotor();
		Serial.print(" Count 1: ");
		Serial.print(motor1QeiCounts);
		Serial.print(" Count 2: ");
		Serial.println(motor2QeiCounts);
		motor1Wheel.setMotorPower(-motor1Speed);
		motor2Wheel.setMotorPower(-motor2Speed);
		delay(1000);
	}
	motor1Wheel.stopMotor();
	motor2Wheel.stopMotor();
}


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
