//#include <Wire.h>
#include "pins.h"
#include <Servo.h>

//#include "Mag3110_v10.pde"
#include "servo_x.h"
#include "cycle_check.h"

#define CHASSIS_ADDR 0x13 //19

#define I2C_Addr 19
#define CMD_LEFT 'l'
#define CMD_RIGHT 'r'
#define CMD_BOTH 'b'
#define CMD_STOP 's'
#define CMD_BATTERY 'v'
#define CMD_CHARGER 'c'
#define CMD_CURRENT 'e'
#define CMD_LED 'd'
#define CMD_OTHER ':'

//movement queue
//function: stop movement and decide

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

int battery_led_state = 0;

bool led_state = 0;

ServoX head_servo, baseturn_servo, basetilt_servo;

//right
void motorRight(int pwm, boolean reverse = false)
{
	if (pwm < 0) {
		pwm = -pwm;
		reverse = !reverse;
	}
	analogWrite(RIGHT_PWM_PIN, pwm);
	//set pwm control, 0 for stop, and 255 for maximum speed
	if (reverse) {
		digitalWrite(RIGHT_DIR_PIN, HIGH);
	} else {
		digitalWrite(RIGHT_DIR_PIN, LOW);
	}
}

//left
void motorLeft(int pwm, boolean reverse = false)
{
	if (pwm < 0) {
		pwm = -pwm;
		reverse = !reverse;
	}
	analogWrite(LEFT_PWM_PIN, pwm);
	if (reverse) {
		digitalWrite(LEFT_DIR_PIN, HIGH);
	} else {
		digitalWrite(LEFT_DIR_PIN, LOW);
	}
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

int getIntVal(const String &strval)
{
	int rs=0;
	int signm = 1;
	int i=0;
	while(i<strval.length() && strval.charAt(i) != ';' && strval.charAt(i) != 0) {
		char b = strval.charAt(i++);
		if(byte == ';') {
			break;
		}
		if(i == 0 && b == '-') {
			signm = -1;
		} else {
			rs = b - '0' + rs * 10;
		}
	}
	rs *= signm;
	Serial.print("value:");
	Serial.println(rs);
	return rs;
}

void executeCommand(String c_args[], int n)
{
	String rs("Command unknown");
	char cmd = c_args[0].charAt(0);
	int nparm = 1;
	Serial.print("command:");
	Serial.println(cmd);
	switch (cmd) {
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
		case CMD_OTHER:
			for(int ci=0; ci< sizeof(scommands)/sizeof(scommands[0]); ci++) {
				if(c_args[0].equals(scommands[ci].cmdstr)) {
					int icmd = scommands[ci].cmd;
					switch(icmd) {
						case CMD_SHOWCANDLE:
							basetilt_servo.setAngle(90, 20);
							rs = "Ok";
							break;
						case CMD_SHOWGE:
							basetilt_servo.setAngle(30, 20);
							break;
						case CMD_SETBASETURNANGLE:
							baseturn_servo.setAngle(getIntVal(c_args[1]), getIntVal(c_args[2]));
							break;
						default:
//							beep();
							break;
					}
				}
			}
			break;
	}
	sendResponse(rs);
}

void setup()
{
	Serial.begin(9600);
	pinMode(LEFT_PWM_PIN, OUTPUT);
	pinMode(LEFT_DIR_PIN, OUTPUT);
	pinMode(RIGHT_PWM_PIN, OUTPUT);
	pinMode(RIGHT_DIR_PIN, OUTPUT);
	pinMode(BATTERY_LED_PIN, OUTPUT);
	pinMode(CURRENT_LED_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
	head_servo.attach(HEAD_PWM_PIN);
	head_servo.setAngle(90, 5);
	basetilt_servo.attach(BASE_TILT_PWM_PIN);
	basetilt_servo.setAngle(90, 30);
	baseturn_servo.attach(BASE_TURN_PWM_PIN);
	baseturn_servo.setAngle(90, 10);
	//setup arm in the initial position
//	Wire.begin(I2C_Addr);
	//join i2c bus
//	Wire.onReceive(receiveEvent);
//	Wire.onRequest(requestData);
	//mag_config();
}

#define MAX_CMDVALS 10

int processCmdline(String &cmdline, String c_args[], int n)
{
	String val;
	int lastpos=0, pos=0, i=0;
	while(pos >= 0 && i < n) {
		pos = cmdline.indexOf(';', lastpos);
		if(pos < 0) {
			//the last value
			val = String(cmdline.substring(lastpos));
		} else {
			val = String(cmdline.substring(lastpos, pos));
			lastpos = pos + 1;
		}
		if(val.length() > 0) {
			c_args[i++] = val;
		}
	}
	for(int j = i;j < n; j++) {
		c_args[j] = "";
	}
	return i
}

void loop()
{
	String c_args[MAX_CMDVALS];
	//test();
	//mag_print_values();
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
		int nc = processCmdline(cmdbuf, c_args, MAX_CMDVALS);
		if(nc > 0) {
			executeCommand(c_args, nc);
		}
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
	static unsigned long	led_timer_state;
	if (cycleCheck(&led_timer_state, 500)) {
		digitalWrite(LED_PIN, led_state);
		led_state = !led_state;
	}
	if (baseturn_servo.update()) {
		if (baseturn_servo.read() == 90) {
			baseturn_servo.setAngle(0);
		} else if (baseturn_servo.read() == 0) {
			baseturn_servo.setAngle(180);
		} else {
			baseturn_servo.setAngle(90);
                }
  	}
	if (basetilt_servo.update()) {
		if (basetilt_servo.read() == 90) {
			basetilt_servo.setAngle(0);
		} else if (basetilt_servo.read() == 0) {
			basetilt_servo.setAngle(180);
		} else {
			basetilt_servo.setAngle(90);
}
	}
//	Serial.println(baseturn_servo.read());
//	Serial.println(basetilt_servo.read());
	head_servo.update();
//        delay(100);
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

