#include <ros.h>
#include <oculus2wd/drive.h>
#include <oculus2wd/drive_status.h>

#include "../../include/common.h"
#include "../../include/printf.h"
#include <Servo.h>

/*
 oculusDC version 0.5.5
 ASCII Serial Commands
 All 2 byte pairs, except for STOP, GET_VERSION, and CAMRELEASE

 FORWARD = 'f', [0-255] (speed)
 BACKWARD = 'b', [0-255] (speed)
 LEFT = 'l', [0-255] (speed)
 RIGHT = 'r', [0-255] (speed)
 CAM = 'v', [0-255] (servo angle)
 STOP = 's' (DC motors stop)
 CAMRELEASE = 'w'
*/

// pins
// H-bridge pin 2				 LEFT motor
#define motorA1Pin 4
// H-bridge pin 7				 LEFT motor
#define motorA2Pin 2
// H-bridge pin 10				 RIGHT motor
#define motorB1Pin 7
// H-bridge pin 15				 RIGHT motor
#define motorB2Pin 8
// H-bridge enable pin 9	 LEFT motor
#define enablePinA 3
// H-bridge enable pin 1	 RIGHT motor
#define enablePinB 11
//camera servo pin
#define camservopin 6

Servo camservo; // tilt

// DC motor compensation
int8_t acomp = 0;
int8_t bcomp = 0;

ros::NodeHandle	 nh;

oculus2wd::drive_status status_msg;

ros::Publisher reply("/oculus2wd/base_status", &status_msg);

void reply_status()
{
	status_msg.left = OCR2A;
	status_msg.right = OCR2B;
	status_msg.camservo = camservo.read();
	status_msg.fw_version = "ros_oculus_base:0.1";
	reply.publish( &status_msg );
}

// do multi byte
void parseCommand(char cmd, int power, int secs)
{
	// always set speed on each move command
	if((cmd == 'f') || (cmd == 'b') || (cmd == 'l') || (cmd == 'r'))
	{
		OCR2A =	 power - acomp*( (float) power / 254.0);
		OCR2B =	 power - bcomp*( (float) power / 254.0);
	}

	switch(cmd) {
		case 'f': // forward
			digitalWrite(motorA1Pin, HIGH);
			digitalWrite(motorA2Pin, LOW);
			digitalWrite(motorB1Pin, HIGH);
			digitalWrite(motorB2Pin, LOW);
			break;
		case 'b': // backward
			digitalWrite(motorA1Pin, LOW);
			digitalWrite(motorA2Pin, HIGH);
			digitalWrite(motorB1Pin, LOW);
			digitalWrite(motorB2Pin, HIGH);
			break;
		case 'r': // right
			digitalWrite(motorA1Pin, HIGH);
			digitalWrite(motorA2Pin, LOW);
			digitalWrite(motorB1Pin, LOW);
			digitalWrite(motorB2Pin, HIGH);
			break;
		case 'l': // left
			digitalWrite(motorA1Pin, LOW);
			digitalWrite(motorA2Pin, HIGH);
			digitalWrite(motorB1Pin, HIGH);
			digitalWrite(motorB2Pin, LOW);
			break;
		case 's': // stop
			digitalWrite(motorA1Pin, HIGH);
			digitalWrite(motorA2Pin, HIGH);
			digitalWrite(motorB1Pin, HIGH);
			digitalWrite(motorB2Pin, HIGH);
			OCR2A = 0;
			OCR2B = 0;
			break;
		case 'v': // camtilt
			camservo.attach(camservopin);
			camservo.write(power);
			break;
		case 'w': // camrelease
			camservo.detach();
			break;
	}
	reply_status();
}

void messageCb(const oculus2wd::drive& command_msg){
	parseCommand(command_msg.command, command_msg.power, command_msg.secs);
}

ros::Subscriber<oculus2wd::drive> request("/oculus2wd/base_command", messageCb );

void setup()
{
	nh.initNode();
	nh.advertise(reply);
	nh.subscribe(request);
	nh.spinOnce();
}

void loop() {
	nh.spinOnce();
	if( !nh.connected() )
	{
		// if no comm with host, stop motors
		OCR2A = 0;
		OCR2B = 0;
		camservo.detach();
	}
}
