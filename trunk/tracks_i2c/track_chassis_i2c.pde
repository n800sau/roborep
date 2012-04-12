//chassis address - 19
#include <Wire.h>

#define I2C_Addr 19
#define CMD_LEFT 'l'
#define CMD_RIGHT 'r'
#define CMD_BOTH 'b'
#define CMD_STOP 's'
#define CMD_BATTERY 'v'
#define CMD_CHARGER 'c'
#define CMD_LED 'd'

//1024 = 10v
#define BATTERY_DIV (1024 / 10.)

//1024 = 30v
#define CHARGER_DIV (1024 / 30.)

#define LEFT_PIN 10
#define RIGHT_PIN 12

#define LED_PIN 14

char cmd = NULL;

void setup()
{
	pinMode(LEFT_PIN, OUTPUT);
	pinMode(RIGHT_PIN, OUTPUT);
	Wire.begin(I2C_Addr); // join i2c bus
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestData);
}

void loop()
{
	float val = get_battery();
	if(val < 6) {
		Wire.beginTransmission(4); // transmit to device #4
		Wire.write('w');
		Wire.write("Battery is low");
		Wire.endTransmission();    // stop transmitting
	} else if (val < 5) {
		Wire.beginTransmission(4); // transmit to device #4
		Wire.write('e');
		Wire.write("Battery is flat");
		Wire.endTransmission();    // stop transmitting
	}
	delay(500);
}

float get_battery()
{
	return analogRead(BATTERY_PIN) / BATTERY_DIV;
}

float get_charger()
{
	return analogRead(CHARGER_PIN) / CHARGER_DIV;
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
		rs = Wire.read() - '0' + rs * 10;
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
	}
}

