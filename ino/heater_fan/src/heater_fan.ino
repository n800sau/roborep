#include "ACS712.h"

ACS712 sensor(ACS712_05B, A0);

const int fan = 2;
const int heater = 3;

const int a_pin = A0;

void setup()
{
	Serial.begin(115200);
	pinMode(fan, OUTPUT);
	digitalWrite(fan, LOW);
	pinMode(heater, OUTPUT);
	digitalWrite(heater, LOW);
	Serial.println("Calibrating... Ensure that no current flows through the sensor at this moment");
	int zero = sensor.calibrate();
	Serial.println("Done!");
	Serial.print("Zero point for this sensor = ");
	Serial.println(zero);
	for(int i=0; i<2; i++) {
		Serial.print("No current:");
		Serial.println(sensor.getCurrentDC());
		delay(1000);
	}
}


void loop()
{
	digitalWrite(heater, HIGH);
	for(int i=0; i<12; i++) {
		Serial.print("Heater:");
		Serial.println(sensor.getCurrentDC());
		delay(5000);
	}
	digitalWrite(heater, LOW);
	digitalWrite(fan, HIGH);
	for(int i=0; i<6; i++) {
		Serial.print("Fan:");
		Serial.println(sensor.getCurrentDC());
		delay(5000);
	}
	digitalWrite(fan, LOW);
}
