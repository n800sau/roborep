#include <Arduino.h>

#ifndef LED_BUILTIN
		#define LED_BUILTIN PC13
#endif

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
//	Serial1.begin(9600);
//	Serial2.begin(9600);
//	Serial3.begin(9600);
}

void loop()

{
	digitalWrite(LED_BUILTIN, 1);
//	Serial1.println("Serial LED OFF");
	Serial.println("Serial zero LED OFF");
//	Serial2.println("Serial DUE LED OFF");
//	Serial3.println("Serial TRE LED OFF");

	delay(1000);

	digitalWrite(LED_BUILTIN, 0);
//	Serial1.println("Serial LED ON");
	Serial.println("Serial zero LED ON");
//	Serial2.println("Serial DUE LED ON");
//	Serial3.println("Serial TRE LED ON");

	delay(1000);
}
