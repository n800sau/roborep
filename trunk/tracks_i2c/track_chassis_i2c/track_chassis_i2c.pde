#include <Wire.h>
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

char cmd = NULL;

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
	float		val = get_current();
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

void baseturn_servo_set(int angle)
{
	baseturn_servo.write(angle);
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
	Wire.begin(I2C_Addr);
	//join i2c bus
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestData);
	//mag_config();
}

void serialEvent()
{
	while(Serial.available()) {
		int serialData = Serial.read();
		Serial.println(serialData);
		Serial.flush();
	}
}

void loop()
{
	//test();
	//mag_print_values();
	float val = get_battery();
	//Serial.print("b:");
	//Serial.print(val);
	if (val < 6) {
		battery_led_state = !battery_led_state;
		Wire.beginTransmission(4);
		//transmit to device #4
		Wire.send('w');
		Wire.send("Battery is low");
		Wire.endTransmission();
		//stop transmitting
	} else if (val < 5) {
		battery_led_state = 1;
		Wire.beginTransmission(4);
		//transmit to device #4
		Wire.send('e');
		Wire.send("Battery is flat");
		Wire.endTransmission();
		//stop transmitting
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

//input val - string with optional '-' in the beginning
int readVal()
{
	int rs = 0;
	int signm = 1;
	char b = Wire.receive();
	if (b == '-') {
		signm = -1;
	} else {
		rs = b - '0';
	}
	while (Wire.available()) {
		b = Wire.receive();
		if (b == ';')
			break;
		rs = b - '0' + rs * 10;
	}
	rs *= signm;
	Serial.print("value:");
	Serial.println(rs);
	return rs;
}

//function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
	int av = Wire.available();
	cmd = Wire.receive();
	Serial.print("command:");
	Serial.println(cmd);
	switch (cmd) {
		//run left motor
		case CMD_LEFT:
			motorLeft(readVal());
			break;
			//run right motor
		case CMD_RIGHT:
			motorRight(readVal());
			break;
			//run two motors
		case CMD_BOTH:
			motorLeft(readVal());
			motorRight(readVal());
			break;
			//stop two motors
		case CMD_STOP:
			motorLeft(0);
			motorRight(0);
			break;
			//make buildin led light
		case CMD_LED:
			analogWrite(LED_PIN, readVal());
			break;
	}
}

void requestData()
{
	float val;
	char buf[50];
	switch (cmd) {
		case CMD_BATTERY:
			val = get_battery();
			Serial.print("battery:");
			Serial.println(val);
			sprintf(buf, "%f", val);
			Wire.send(buf);
			break;
		case CMD_CHARGER:
			val = get_charger();
			Serial.print("charger:");
			Serial.println(val);
			sprintf(buf, "%f", val);
			Wire.send(buf);
			break;
		case CMD_CURRENT:
			float		val = get_current();
			Serial.print("current:");
			Serial.println(val);
			sprintf(buf, "%f", val);
			Wire.send(buf);
			break;
	}
}
