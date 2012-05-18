#include <Wire.h>
#include <Mag3110_v10.h>

#include "pins.h"
#include "cycle_check.h"
#include "serial_cmd.h"
#include "servo_x.h"

#include "chassis.h"
#include "movements.h"
#include "commands.h"

#define I2C_Addr 0x13 //19
#define I2C_TopModule_Addr 0x14 //20

ServoX head_servo;

int battery_led_state = 0;

bool led_state = 0;

#define MAX_CMDARGS 10

void errorBeep()
{
}

float get_battery()
{
	return analogRead(BATTERY_PIN) / 1024. * 24.9;
	//divider 21.6 / 82.4 = 0.262 1.84 / 8.94 = 0.205 5 / 0.205 = 24.39 5 / 0.262 =
}

float get_charger()
{
	return analogRead(CHARGER_PIN) / 1024. * 41.5;
	//divider 12 / 82.4
}

float get_current()
{
	return 13.12 - analogRead(CURRENT_SENSOR_PIN) / 1024. * 5 / 0.185;
}

void test()
{
	motorLeft(200, 0);
	motorRight(200, 0);
	float val = get_current();
	Serial.print(", curr:");
	Serial.println(val);
	delay(1000);
	motorLeft(200, 1);
	motorRight(200, 1);
	val = get_current();
	Serial.print(", curr:");
	Serial.println(val);
	delay(1000);
	motorLeft(0);
	motorRight(0);
	val = get_current();
	Serial.print(", curr:");
	Serial.println(val);
}

String executeCommand(String c_args[], int n)
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
					//run left motor
					case CMD_LEFT:
						motorLeft(getIntVal(c_args[1]));
						break;
					//run right motor
					case CMD_RIGHT:
						motorRight(getIntVal(c_args[1]));
						break;
						//run two motors
					case CMD_BOTH:
						motorLeft(getIntVal(c_args[1]));
						motorRight(getIntVal(c_args[2]));
						break;
						//stop two motors
					case CMD_STOP:
						motorLeft(0);
						motorRight(0);
						break;
						//make buildin led light
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
					case CMD_SETBASETURN:
						baseturn_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
						break;
					case CMD_SETBASETILT:
						basetilt_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
						break;
					case CMD_SETMIDDLETILT:
						middletilt_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
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
		Wire.println(I2C_answer);
		I2C_answer = "";
	}
}

void setup()
{
	serialSetup(I2C_Addr);
	//join i2c bus 
	Wire.onReceive(I2C_receiveEvent);
	Wire.onRequest(I2C_requestData);
	chassisSetup();
	movementsSetup();
	head_servo.attach(HEAD_PWM_PIN);
	//pins
	pinMode(BATTERY_LED_PIN, OUTPUT);
	pinMode(CURRENT_LED_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
	set_movement("base");
}

void loop()
{
	String c_args[MAX_CMDARGS];
	int nc = readSerial(c_args, MAX_CMDARGS);
	if(nc > 0) {
		String answer = executeCommand(c_args, nc);
		Serial.println(answer);
	}
	float val = get_battery();
	//Serial.print("b:");
	//Serial.print(val);
	if (val < 6) {
		battery_led_state = !battery_led_state;
		Serial.println("Battery is low");
	} else if (val < 5) {
		battery_led_state = 1;
		Serial.println("Battery is flat");
	} else {
		battery_led_state = 0;
	}
	val = get_charger();
	//Serial.print(", c:");
	//Serial.println(val);
	digitalWrite(BATTERY_LED_PIN, battery_led_state);
	static unsigned long led_timer_state;
	if (cycleCheck(&led_timer_state, 500)) {
		digitalWrite(LED_PIN, led_state);
		led_state = !led_state;
	}
	if (movementsUpdate()) {
		Serial.print(current_move->name);
		Serial.println(" finished");
		if (current_move->name == "base") {
			set_movement("candle");
		} else {
			set_movement("base");
		}
  	}
//	Serial.println(baseturn_servo.read());
//	Serial.println(basetilt_servo.read());
	head_servo.update();
//        delay(100);
}
