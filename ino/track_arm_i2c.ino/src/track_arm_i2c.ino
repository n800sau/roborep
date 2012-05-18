#include <Servo.h>

#include <Wire.h>
#include <QTRSensors.h>
#include <ADXL345.h>

#include "pins.h"
#include "servo_x.h"
#include "cycle_check.h"
#include "serial_cmd.h"
#include "movements.h"
#include "commands.h"

#define I2C_Addr 0x14 //20
#define I2C_LowModule_Addr 0x13 //19

#define MAX_CMDARGS 10

#define QTRNUM 1
byte pins[] = {QTR1_PIN};
QTRSensorsAnalog trsensors(pins, QTRNUM);
ADXL345 Accel;

bool led_state = false;

void errorBeep()
{
}

String executeCommand(String c_args[], int n)
{
	String rs("Command unknown");
	Serial.print("command:");
	Serial.println(c_args[0]);
	for(int ci=0; ci< sizeof(scommands)/sizeof(scommands[0]); ci++) {
		if(c_args[0].equals(scommands[ci].cmdstr)) {
			if(scommands[ci].n_parms != n && scommands[ci].n_parms >= 0) {
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
					case CMD_MOVEMENT_STOP:
						movement_stop();
						break;
					case CMD_CHASSIS:
						Wire.beginTransmission(I2C_LowModule_Addr); // transmit to Low Module
						for(int k=1; k<n; k++) {
							Wire.print(c_args[k]);
							Wire.write(( k < n-1 ) ? ";" : "\n");
						}
						Wire.endTransmission();       // stop transmitting
						Wire.requestFrom(I2C_LowModule_Addr, 100);
						while(Wire.available()) // slave may send less than requested
						{
							byte b = Wire.read(); // receive a byte as character
							Serial.print(b); // print the character
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
	return rs;
}

void setup()
{
	serialSetup(I2C_Addr);
	movementsSetup();
	pinMode(LED_PIN, OUTPUT);
}

void loop()
{
	String c_args[MAX_CMDARGS];
	int nc = readSerial(c_args, MAX_CMDARGS);
	if(nc > 0) {
		String answer = executeCommand(c_args, nc);
		Serial.println(answer);
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
		double acc_data[3];
		Accel.get_Gxyz(acc_data);
		if(Accel.status){
		    float length = 0.;
		    for(int i = 0; i < 3; i++){
			length += (float)acc_data[i] * (float)acc_data[i];
			Serial.print(acc_data[i]);
			Serial.print(" ");
		    }
		    length = sqrt(length);
		    Serial.print(length);
		    Serial.println("");
	    } else {
		Serial.println("ERROR: ADXL345 data read error");
	    }
	}
	if (movementsUpdate()) {
		Serial.print(current_move->name);
		Serial.println(" finished");
		if (current_move->name == "openclaw") {
			set_movement("closeclaw");
		} else {
			set_movement("openclaw");
		}
  	}
}


String I2C_answer = "";

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void I2C_receiveEvent(int howMany)
{
	String c_args[MAX_CMDARGS];
	int nc = readI2C(c_args, MAX_CMDARGS);
	if(nc > 0) {
		I2C_answer = executeCommand(c_args, nc);
	}
}

void I2C_requestData()
{
	if(I2C_answer != "") {
		Wire.print(I2C_answer);
		I2C_answer = "";
	}
}
