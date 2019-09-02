#include <EventFuse.h>
#include <MsTimer2.h>
#include <JsonParser.h>
#include "../../include/common.h"
#include "../../include/printf.h"
#include <Servo.h>

/*
	{'command': 'f', 'power': 100, 'secs': 1}
	JSON commands:

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

void reply_status()
{
	printf("{\"left\":%d,\"right\":%d,\"camservo\":%d}\r\n", OCR2A, OCR2B, camservo.read());
}

// do multi byte
void executeCommand(char cmd, int power, int secs)
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

void fuseEvent(FuseID fuse, int &userData) {
	reply_status();
}

void timerTick(){
	EventFuse::burn(1);
}

void setup()
{
	Serial.begin(57600);
	printf_begin();
	OCR2A = 0;
	OCR2B = 0;
	camservo.detach();
	EventFuse::newFuse(10, INF_REPEAT, fuseEvent);
	// 1 second fuse unit
	MsTimer2::set(1000, timerTick);
	MsTimer2::start();
}

#define MAX_INPUT_LEN 200
char inputString[MAX_INPUT_LEN];
int inputPos = 0;
boolean stringComplete = false;
void serialEvent() {
	while (Serial.available()) {
		char inChar = (char)Serial.read();
		// add it to the inputString:
		inputString[inputPos++] = inChar;
		inputString[inputPos] = 0;
		if (inChar == '\n' || inputPos >= MAX_INPUT_LEN-1) {
			stringComplete = true;
		}
	}
}

void loop() {
	if (stringComplete) {
		JsonParser<32> parser;
		JsonHashTable data = parser.parseHashTable(inputString);
		char* cmd = data.getString("command");
		int power = data.getLong("power");
		int secs = data.getLong("secs");
		executeCommand(cmd[0], power, secs);
		inputString[0] = 0;
		stringComplete = false;
	}
}
