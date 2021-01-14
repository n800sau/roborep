/*

am2320
sda - A4
scl - A5

heater pwm - 3

*/

#include <AM2320.h>

AM2320 am2320;

void setup()
{
	Serial.begin(115200);
	am2320.begin();
}

void loop()
{
	if (am2320.measure()) {
		Serial.print("Temperature: ");
		Serial.println(am2320.getTemperature());
		Serial.print("Humidity: ");
		Serial.println(am2320.getHumidity());
	}
	else
	{
		int errorCode = am2320.getErrorCode();
		switch (errorCode)
		{
			case 1: Serial.println("ERR: am2320 is offline"); break;
			case 2: Serial.println("ERR: CRC validation failed."); break;
		}
	}
	delay(500);
}
