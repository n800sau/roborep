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
#define BATTERY_DIV (1024 / 10) 

//1024 = 30v
#define CHARGER_DIV (1024 / 30) 

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
	Wire.beginTransmission(4); // transmit to device #4
	Wire.write("x is ");        // sends five bytes
	Wire.write(x);              // sends one byte  
	Wire.endTransmission();    // stop transmitting
	delay(500);
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
			int val = analogRead(BATTERY_PIN);
			Serial.print("battery:");
			Serial.println(val);
			char buf[3 * sizeof (int) + 1];
			itoa(val / BATTERY_DIV, buf, 10);
			Wire.write(buf);
			break;
		case CMD_CHARGER:
			int val = analogRead(CHARGER_PIN);
			Serial.print("charger:");
			Serial.println(val);
			char buf[3 * sizeof (int) + 1];
			itoa(val / CHARGER_DIV, buf, 10);
			Wire.write(buf);
			break;
	}
}

