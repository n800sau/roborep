#include <SoftwareSerial.h>

#if defined(ESP8266)
SoftwareSerial sSer(D3, -1); // RX, TX
#else
SoftwareSerial sSer(10, 7); // RX, TX
#endif

void setup() {
	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	}


	Serial.println("pwm picaxe test!");

	// set the data rate for the SoftwareSerial port
	sSer.begin(4800);
	sSer.print("Pwm Command");

}

void loop()
{
	for(int i=1; i<=4; i++) {
		Serial.print("pwm on:");
		Serial.println(i);
		sSer.print("PWMP");
		sSer.println(i);
		delay(10);
		sSer.println("PER128");
		delay(10);
		sSer.println("DUT1000");
		delay(2000);
		Serial.print("pwm off:");
		Serial.println(i);
		sSer.print("PWMS");
		sSer.println(i);
		delay(2000);
	}
}

