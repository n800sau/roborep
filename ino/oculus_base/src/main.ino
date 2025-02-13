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
 ANALOG WRITE = 'a', [pin #] [0-255] (analogwrite to pin)
 DIGITAL READ = 'd', [pin #] (digitalread pin, send result via serial)
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

// buffer the command in byte buffer
const int MAX_BUFFER = 8;
int buffer[MAX_BUFFER];
int commandSize = 0;
unsigned long lastcmd = 0;
int timeout = 50000;

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

	Serial.begin(57600);
	Serial.println("<reset>");
	lastcmd = millis();
}

void loop()
{
	if( Serial.available() > 0 )
	{
		// commands take priority
		lastcmd = millis();
		manageCommand();
	}
	if (millis() - lastcmd > timeout)
	{
		// if no comm with host, stop motors
		lastcmd = millis();
		OCR2A = 0;
		OCR2B = 0;
		camservo.detach();
	}
}

// buffer and/or execute commands from host controller
void manageCommand()
{
	int input = Serial.read();

	// end of command -> exec buffered commands
	if((input == 13) || (input == 10))
	{
		if(commandSize > 0)
		{
			parseCommand();
			commandSize = 0;
		}
	}
	else
	{
		// buffer it
		buffer[commandSize++] = input;

		// protect buffer
		if(commandSize >= MAX_BUFFER)
		{
			commandSize = 0;
		}
	}
}

void reply_status()
{
	Serial.print(", l:");
	if(digitalRead(motorA2Pin) == HIGH) {
		Serial.print("-");
	}
	Serial.print(OCR2A);
	Serial.print(", r:");
	if(digitalRead(motorB2Pin) == HIGH) {
		Serial.print("-");
	}
	Serial.print(OCR2B);
	Serial.print(", servo:");
	Serial.println(camservo.read());
}

// do multi byte
void parseCommand()
{
	Serial.println(REPLY_START_MARKER);
	int mB, mA, n, pA, pB;
	// always set speed on each move command
	if((buffer[0] == 'f') || (buffer[0] == 'b') || (buffer[0] == 'l') || (buffer[0] == 'r'))
	{
		OCR2A =	 buffer[1] - acomp*( (float) buffer[1] / 254.0);
		OCR2B =	 buffer[1] - bcomp*( (float) buffer[1] / 254.0);
		// Serial.println("<speed " + (String)buffer[1] + ">");
	}

	switch(buffer[0]) {
		case 'm':
			mA = buffer[1]&0x80;
			mB = buffer[2]&0x80;

			pA = (buffer[1]&0x7f)<<1;
			pB = (buffer[2]&0x7f)<<1;

			if(mA) {
				pA = (pA ^ 0xff) + 1;
			}

			if(mB) {
				pB = (pB ^ 0xff) + 1;
			}

//	Serial.print(pA);
//	Serial.print(":");
//	Serial.println(pB);

//	Serial.print(mA);
//	Serial.print(":");
//	Serial.println(mB);

			OCR2A = pA;
			OCR2B = pB;

			if (mB) {
				digitalWrite(motorB1Pin, LOW);
				digitalWrite(motorB2Pin, HIGH);
			} else {
				digitalWrite(motorB1Pin, HIGH);
				digitalWrite(motorB2Pin, LOW);
			}

			if (mA) {
				digitalWrite(motorA1Pin, LOW);
				digitalWrite(motorA2Pin, HIGH);
			} else {
				digitalWrite(motorA1Pin, HIGH);
				digitalWrite(motorA2Pin, LOW);
			}
			reply_status();
			break;
		case 'a':
			n = buffer[2]&255;
			analogWrite((int) buffer[1], n);
			break;
		case 'd':
			Serial.println("<digital pin " + (String) buffer[1] + ": " + (String) digitalRead(buffer[1]) + ">");
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
			camservo.write(buffer[1]);
			reply_status();
			break;
		case 'w': // camrelease
			camservo.detach();
			reply_status();
			break;
		case 'c':
			// 128 = 0, > 128 = acomp, < 128 = bcomp
			if (buffer[1] == 128)
			{
				acomp = 0;
				bcomp = 0;
			}
			if (buffer[1] > 128)
			{
				bcomp = 0;
				acomp = (buffer[1]-128)*2;
			}
			if (buffer[1] < 128)
			{
				acomp = 0;
				bcomp = (128-buffer[1])*2;
			}
			reply_status();
			break;
		case 'e':
			if(buffer[1] == '1')
				echo = true;
			if(buffer[1] == '0')
				echo = false ;
			Serial.print("echo ");
			Serial.println((echo)?"on":"off");
			break;
		case 'x':
			Serial.println("<id:oculusDC>");
			break;
		case 'y':
			Serial.println("<version:0.5.5>");
			break;
	}

	// echo the command back
	if(echo)
	{
		Serial.print("<");
		Serial.print((char)buffer[0]);

		if(commandSize > 1)
			Serial.print(',');

		for(int b = 1 ; b < commandSize ; b++)
		{
			Serial.print((String)buffer[b]);
			if(b<(commandSize-1))
				Serial.print(',');
		}
		Serial.println(">");
	}
	Serial.println(END_MARKER);
}
