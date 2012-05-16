#include <Wire.h>
#include <QTRSensors.h>

#include "pins.h"
#include "servo_x.h"
#include "cycle_check.h"
#include "serial_cmd.h"
#include "movements.h"
#include "commands.h"

#define I2C_Addr 0x14 //20
#define I2C_LowModule_Addr 0x13 //19

#define I2C_OK_REQUEST '?'

#define MAX_CMDARGS 10

#define QTRNUM 1
QTRSensorsAnalog trsensors({QTR1_PIN}, QTRNUM);

bool led_state = false;

void executeCommand(String c_args[], int n)
{
	String rs("Command unknown");
	Serial.print("command:");
	Serial.println(c_args[0]);
	for(int ci=0; ci< sizeof(scommands)/sizeof(scommands[0]); ci++) {
		if(c_args[0].equals(scommands[ci].cmdstr)) {
			if(scommands[ci].n_parms != n) {
				errorBeep();
			} else {
				int icmd = scommands[ci].cmd;
				switch(icmd) {
					case CMD_LED:
						analogWrite(LED_PIN, getIntVal(c_args[1]));
						break;
					case CMD_MOVEMENT:
						if(!set_movement(c_args[1])) {
							errorBeep();
							rs = "Error";
						} else {
							rs = "Ok";
						}
						break;
					case CMD_SETPALMTURN:
						palmturn_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
						break;
					case CMD_SETPALMTILT:
						palmtilt_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
						break;
					case CMD_SETCLAW:
						claw_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
						break;
					default:
						errorBeep();
						break;
				}
			}
		}
	}
	Serial.println(rs);
}

void setup()
{
	serialSetup(I2C_Addr);
	chassisSetup();
	movementsSetup();
	pinMode(LED_PIN, OUTPUT);
}

void loop()
{
	String c_args[MAX_CMDARGS];
	int nc = readSerial(c_args, MAX_CMDARGS);
	if(nc > 0) {
		executeCommand(c_args, nc);
	}
	static unsigned long led_timer_state = millis();
	if (cycleCheck(&led_timer_state, 500)) {
		digitalWrite(LED_PIN, led_state);
		led_state = !led_state;
	}
	static unsigned long sensors_timer_state = millis();
	if (cycleCheck(&sensors_timer_state, 2000)) {
		unsigned int sensor_values[QTRNUM];
		trsensors.read(sensor_values);
		Serial.print("QTR=");
		Serial.println(sensor_values[0]);
	}
	if (movementsUpdate()) {
		Serial.print(last_move);
		Serial.println(" finished");
		if (last_move && strcmp(last_move, "base")) {
			set_movement("closeclaw");
		} else {
			set_movement("openclaw");
		}
  	}
}

struct {
	char cmd;
} I2C_request = {NULL};

void I2C_receiveEvent(int howMany)
{
	String c_args[MAX_CMDARGS];
	int nc = readI2C(c_args, MAX_CMDARGS);
	if(nc > 0) {
		executeCommand(c_args, nc);
	}
}

void I2C_requestData()
{
	if(I2C_request.cmd) {
		Wire.beginTransmission(I2C_LowModule_Addr); // transmit to Low Module
		Wire.send("ok");
		Wire.endTransmission();       // stop transmitting
		I2C_request.cmd = NULL;
	}
}
