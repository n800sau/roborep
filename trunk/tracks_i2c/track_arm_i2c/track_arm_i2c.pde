#include <Wire.h>

#include "pins.h"
#include "servo_x.h"
#include "cycle_check.h"
#include "serial_cmd.h"

#define I2C_Addr 0x14 //20
#define I2C_LowModule_Addr 0x13 //19

#define CMD_OTHER ':'
#define I2C_OK_REQUEST '?'

#define MAX_CMDARGS 10

enum SCOMMANDS {
	CMD_SHOWCANDLE,
	CMD_SHOWGE,
	CMD_SETBASETURNANGLE,
};

//long string commands
typedef struct {
	const char *cmdstr;
	int cmd;
	int n_parms;
} LONG_CMD;

const LONG_CMD scommands[] = {
	{"candle", CMD_SHOWCANDLE, 0},
	{"ge", CMD_SHOWGE, 0},
	{"setBaseTurnAngle", CMD_SETBASETURNANGLE, 2}
};

ServoX palmturn_servo, palmtilt_servo, claw_servo;

bool led_state = false;

struct {
	char cmd;
} I2C_request = {NULL};


void executeCommand(String c_args[], int n)
{
	String rs("Command unknown");
	char cmd = c_args[0].charAt(0);
	int nparm = 1;
	Serial.print("command:");
	Serial.println(cmd);
	switch (cmd) {
		case I2C_OK_REQUEST:
			I2C_request.cmd = cmd;
			break;
		case CMD_OTHER:
			for(int ci=0; ci< sizeof(scommands)/sizeof(scommands[0]); ci++) {
				if(c_args[0].equals(scommands[ci].cmdstr)) {
					int icmd = scommands[ci].cmd;
					switch(icmd) {
						case CMD_SHOWCANDLE:
							palmtilt_servo.setAngle(90, 20);
							rs = "Ok";
							break;
						case CMD_SHOWGE:
							palmtilt_servo.setAngle(30, 20);
							break;
						case CMD_SETBASETURNANGLE:
							palmturn_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
							break;
						default:
//							beep();
							break;
					}
				}
			}
			break;
	}
	Serial.println(rs);
}

void setup()
{
	Serial.begin(9600);
	pinMode(LED_PIN, OUTPUT);
	palmtilt_servo.attach(PALM_TILT_PWM_PIN);
	palmtilt_servo.setAngle(90, 30);
	palmturn_servo.attach(PALM_TURN_PWM_PIN);
	palmturn_servo.setAngle(90, 10);
	claw_servo.attach(CLAW_PWM_PIN);
	claw_servo.setAngle(90, 10);
	//setup arm in the initial position
	Wire.begin(I2C_Addr);
	//join i2c bus
	Wire.onReceive(I2C_receiveEvent);
	Wire.onRequest(I2C_requestData);
}

void serialRead()
{
	String c_args[MAX_CMDARGS];
	char cmdbuf[20];
	int n = 0;
	while(Serial.available()) {
		int byte = Serial.read();
		if(byte == '\n') {
			byte = 0;
		}
		if(n < sizeof(cmdbuf)) {
			cmdbuf[n++] = byte;
		} else {
			cmdbuf[0] = 0;
			//beep();
		}
//		Serial.println(serialData);
//		Serial.flush();
	}
	if(n > 0 && cmdbuf[0]) {
		int nc = processCmdline(String(cmdbuf), c_args, MAX_CMDARGS);
		if(nc > 0) {
			executeCommand(c_args, nc);
		}
	}
}

void loop()
{
	//test();
	//mag_print_values();
	serialRead();
	static unsigned long led_timer_state;
	if (cycleCheck(&led_timer_state, 500)) {
		digitalWrite(LED_PIN, led_state);
		led_state = !led_state;
	}
	if (palmturn_servo.update()) {
		if (palmturn_servo.read() == 90) {
			palmturn_servo.setAngle(0);
		} else if (palmturn_servo.read() == 0) {
			palmturn_servo.setAngle(180);
		} else {
			palmturn_servo.setAngle(90);
		}
  	}
	if (palmtilt_servo.update()) {
		if (palmtilt_servo.read() == 90) {
			palmtilt_servo.setAngle(0);
		} else if (palmtilt_servo.read() == 0) {
			palmtilt_servo.setAngle(180);
		} else {
			palmtilt_servo.setAngle(90);
		}
	}
	if (claw_servo.update()) {
		if (claw_servo.read() == 90) {
			claw_servo.setAngle(0);
		} else if (claw_servo.read() == 0) {
			claw_servo.setAngle(180);
		} else {
			claw_servo.setAngle(90);
		}
	}
}

void I2C_receiveEvent(int howMany)
{
	String c_args[MAX_CMDARGS];
	char cmdbuf[20];
	int n = 0;
	while(Wire.available()) {
		int byte = Wire.receive();
		if(byte == '\n') {
			byte = 0;
		}
		if(n < sizeof(cmdbuf)) {
			cmdbuf[n++] = byte;
		} else {
			cmdbuf[0] = 0;
			//beep();
		}
	}
	if(n > 0 && cmdbuf[0]) {
		int nc = processCmdline(cmdbuf, c_args, MAX_CMDARGS);
		if(nc > 0) {
			executeCommand(c_args, nc);
		}
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
