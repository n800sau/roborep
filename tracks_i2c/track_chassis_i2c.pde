//chassis address - 19
#include <Wire.h>
#include "pins.h"

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

int battery_led_state = 0

Servo head_servo, baseturn_servo, basetilt_servo;


void setup()
{
	pinMode(LEFT_PIN, OUTPUT);
	pinMode(RIGHT_PIN, OUTPUT);
	pinMode(BATTERY_LED_PIN, OUTPUT);
	head_servo.attach(HEAD_PWM_PIN);
	head_servo.write(90)
	basetilt_servo.attach(BASETILT_PWM_PIN);
	basetilt_servo.write(90)
	baseturn_servo.attach(BASETURN_PWM_PIN);
	baseturn_servo.write(90)
	//setup arm in the initial position
	Wire.begin(I2C_Addr); // join i2c bus
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestData);
}

void loop()
{
	float val = get_battery();
	if(val < 6) {
		battery_led_state = !battery_led_state;
		Wire.beginTransmission(4); // transmit to device #4
		Wire.write('w');
		Wire.write("Battery is low");
		Wire.endTransmission();    // stop transmitting
	} else if (val < 5) {
		battery_led_state = 1;
		Wire.beginTransmission(4); // transmit to device #4
		Wire.write('e');
		Wire.write("Battery is flat");
		Wire.endTransmission();    // stop transmitting
	} else {
		battery_led_state = 0;
	}
	digitalWrite(BATTERY_LED_PIN, battery_lde_state);
	delay(500);
}

float get_battery()
{
	return map(analogRead(BATTERY_PIN), 0, 1023, 0, 10.); //divider 1/2
}

float get_charger()
{
	return map(analogRead(CHARGER_PIN), 0, 1023, 0, 30.); //divider 1/6
}

float get_current()
{
	return map(analogRead(CURRENT_PIN), 0, 1023, 0, 5/0.18);
}

//input val - string with optional '-' in the beginning
int readVal()
{
	int rs = 0;
	int signm = 1;
	char b = Wire.read();
	if( b == '-') {
		signm = -1;
	} else {
		rs = b - '0';
	}
	while(Wire.available())
	{
		b = Wire.read();
		if(b == ';') break;
		rs = b - '0' + rs * 10;
	}
	rs *= signm;
	Serial.print("value:");
	Serial.println(rs);
	return rs;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
	int av = Wire.available();
	cmd = Wire.read();
	Serial.print("command:");
	Serial.println(cmd);
	switch(cmd) {
		//run left motor
		case CMD_LEFT:
			analogWrite(LEFT_PIN, readVal());
			break;
		//run right motor
		case CMD_RIGHT:
			analogWrite(RIGHT_PIN,readVal());
			break;
		//run two motors
		case CMD_BOTH:
			analogWrite(LEFT_PIN, readVal());
			analogWrite(RIGHT_PIN, readVal());
			break;
		//stop two motors
		case CMD_STOP:
			analogWrite(LEFT_PIN, 0);
			analogWrite(RIGHT_PIN, 0);
			break;
		case CMD_LED:
			analogWrite(LED_PIN, readVal());
			break;
	}
}

void requestData()
{
	switch(cmd) {
		case CMD_GETBATTERY:
			float val = get_battery();
			Serial.print("battery:");
			Serial.println(val);
			char buf[50];
			sprintf(buf, "%f", val);
			Wire.write(buf);
			break;
		case CMD_CHARGER:
			float val = get_charger();
			Serial.print("charger:");
			Serial.println(val);
			char buf[50];
			sprintf(buf, "%f", val);
			Wire.write(buf);
			break;
		case CMD_CURRENT:
			float val = get_current();
			Serial.print("current:");
			Serial.println(val);
			char buf[50];
			sprintf(buf, "%f", val);
			Wire.write(buf);
			break;
	}
}

