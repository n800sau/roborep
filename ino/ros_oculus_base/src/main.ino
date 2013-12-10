#include <ros.h>
#include <oculus2wd/drive.h>
#include <std_msgs/String.h>

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
 COMP = 'c', [0 - 255] (DC motor comp: <128 is left, 128 is none, >128 is right)
 CAM = 'v', [0-255] (servo angle)
 ECHO_ON = 'e', '1' (echo command back TRUE)
 ECHO_OFF = 'e', '0' (echo command back FALSE)
 STOP = 's' (DC motors stop)
 GET_VERSION = 'y'
 CAMRELEASE = 'w'
 DIRECT DIFFERENTIAL STEERING = 'm', [0-255][0-255] (speed motor L&R, <128 is back, 128 is stop, >128 is fwd)
*/

// pins
const int motorA1Pin = 4;		 // H-bridge pin 2				 LEFT motor
const int motorA2Pin = 2;		 // H-bridge pin 7				 LEFT motor
const int motorB1Pin = 7;		 // H-bridge pin 10				 RIGHT motor
const int motorB2Pin = 8;		 // H-bridge pin 15				 RIGHT motor
const int enablePinA = 3;		 // H-bridge enable pin 9	 LEFT motor
const int enablePinB = 11;	 // H-bridge enable pin 1	 RIGHT motor
const int camservopin = 6;

Servo camservo; // tilt

// DC motor compensation
int acomp = 0;
int bcomp = 0;

boolean echo = false;

ros::NodeHandle	 nh;

std_msgs::String str_msg;
ros::Publisher reply("oculus_base_reply", &str_msg);

// do multi byte
void parseCommand(char cmd, int l_power, int r_power, int secs)
{
	char buf[256];
	const char *ptr;
	int mB, mA, n;
	// always set speed on each move command
	if((cmd == 'f') || (cmd == 'b') || (cmd == 'l') || (cmd == 'r'))
	{
		OCR2A =	 l_power - acomp*( (float) l_power / 254.0);
		OCR2B =	 r_power - bcomp*( (float) r_power / 254.0);
	}

	switch(cmd) {
		case 'm':
			mB = l_power&255;
			mA = r_power&255;

			OCR2A =	 (mB&127)<<1;
			OCR2B =	 (mA&127)<<1;

			if (mB<128) {
				digitalWrite(motorB1Pin, LOW);
				digitalWrite(motorB2Pin, HIGH);
			} else {
				digitalWrite(motorB1Pin, HIGH);
				digitalWrite(motorB2Pin, LOW);
			}

			if (mA<128) {
				digitalWrite(motorA1Pin, LOW);
				digitalWrite(motorA2Pin, HIGH);
			} else {
				digitalWrite(motorA1Pin, HIGH);
				digitalWrite(motorA2Pin, LOW);
			}
			reply_status();
			break;
		case 'f': // forward
			digitalWrite(motorA1Pin, HIGH);
			digitalWrite(motorA2Pin, LOW);
			digitalWrite(motorB1Pin, HIGH);
			digitalWrite(motorB2Pin, LOW);
			reply_status();
			break;
		case 'b': // backward
			digitalWrite(motorA1Pin, LOW);
			digitalWrite(motorA2Pin, HIGH);
			digitalWrite(motorB1Pin, LOW);
			digitalWrite(motorB2Pin, HIGH);
			reply_status();
			break;
		case 'r': // right
			digitalWrite(motorA1Pin, HIGH);
			digitalWrite(motorA2Pin, LOW);
			digitalWrite(motorB1Pin, LOW);
			digitalWrite(motorB2Pin, HIGH);
			reply_status();
			break;
		case 'l': // left
			digitalWrite(motorA1Pin, LOW);
			digitalWrite(motorA2Pin, HIGH);
			digitalWrite(motorB1Pin, HIGH);
			digitalWrite(motorB2Pin, LOW);
			reply_status();
			break;
		case 's': // stop
			OCR2A = 0;
			OCR2B = 0;
			reply_status();
			break;
		case 'v': // camtilt
			camservo.attach(camservopin);
			camservo.write(l_power);
			reply_status();
			break;
		case 'w': // camrelease
			camservo.detach();
			reply_status();
			break;
		case 'c':
			// 128 = 0, > 128 = acomp, < 128 = bcomp
			if (l_power == 128)
			{
				acomp = 0;
				bcomp = 0;
			}
			if (l_power > 128)
			{
				bcomp = 0;
				acomp = (l_power-128)*2;
			}
			if (l_power < 128)
			{
				acomp = 0;
				bcomp = (128-l_power)*2;
			}
			reply_status();
			break;
		case 'e':
			echo = l_power != 0;
			snprintf(buf, sizeof(buf),"%s", (echo)?"echo on":"echo off");
			str_msg.data = buf;
			reply.publish( &str_msg );
			break;
		case 'x':
			str_msg.data = "<id:oculusDC>";
			reply.publish( &str_msg );
			break;
		case 'y':
			str_msg.data = "<version:0.5.5>";
			reply.publish( &str_msg );
			break;
	}

	// echo the command back
	if(echo)
	{
		snprintf(buf, sizeof(buf), "received %c %d %d (%d)", cmd, l_power, r_power, secs);
		str_msg.data = buf;
		reply.publish( &str_msg );
	}
}

void messageCb(const oculus2wd::drive& command_msg){
	parseCommand(command_msg.command, command_msg.l_power, command_msg.r_power, command_msg.secs);
}

ros::Subscriber<oculus2wd::drive> request("oculus_base_command", messageCb );

void setup()
{
	pinMode(motorA1Pin, OUTPUT);
	pinMode(motorA2Pin, OUTPUT);
	pinMode(enablePinA, OUTPUT);
	pinMode(motorB1Pin, OUTPUT);
	pinMode(motorB2Pin, OUTPUT);
	pinMode(enablePinB, OUTPUT);
	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20); // phase correct (1/2 freq)
	//TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // 'fast pwm' (1x freq)
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // divide by 1024
	//TCCR2B = _BV(CS22) | _BV(CS20); // divide by 128
	//TCCR2B = _BV(CS21) | _BV(CS20); // divide by 8
	OCR2A = 0;
	OCR2B = 0;

	nh.initNode();
	nh.advertise(reply);
	nh.subscribe(request);
}

void loop()
{
	nh.spinOnce();
	if( !nh.connected() )
	{
		// if no comm with host, stop motors
		OCR2A = 0;
		OCR2B = 0;
		camservo.detach();
	}
	delay(100);
}

void reply_status()
{
	char buf[256];
	snprintf(buf, sizeof(buf), "l: %d, r: %d, servo: %d", OCR2A, OCR2B, camservo.read());
	str_msg.data = buf;
	reply.publish( &str_msg );
}

