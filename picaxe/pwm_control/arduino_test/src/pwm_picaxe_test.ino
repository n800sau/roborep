#include <SoftwareSerial.h>

#if defined(ESP8266)
<<<<<<< HEAD
SoftwareSerial sSer(D7, -1); // RX, TX
#else
//SoftwareSerial sSer(10, 7); // RX, TX
=======
SoftwareSerial sSer(D0, D0); // RX, TX
#else
SoftwareSerial sSer(10, 10); // RX, TX
>>>>>>> b4a3fd92906e721d0c73cb94073d697d9c451abc
#endif

#define PWM_PIN D8
#define PWM_PIN1 D7

void setup() {
	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	Serial.println("Begin");
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	}
	pinMode(PWM_PIN, OUTPUT);
	analogWrite(PWM_PIN, 100);


	Serial.println("pwm picaxe test!");

//	digitalWrite(TEST_PIN, HIGH);
//	delay(2000);
//	digitalWrite(TEST_PIN, LOW);
//	delay(2000);
//	digitalWrite(TEST_PIN, HIGH);
//	delay(2000);

	// set the data rate for the SoftwareSerial port
	sSer.begin(4800);
	sSer.enableTx(true);

}

//int pins[] = {D0, D1, D2, D3, D4, D5, D6, D7, D8};
int pins[] = {D0};
const int pin_count = sizeof(pins)/sizeof(pins[0]);

void loop()
{
	int i;
	for(i=0; i<1024; i++) {
		analogWrite(PWM_PIN, i);
		analogWrite(PWM_PIN1, 1024-i);
		delay(10);
	}
	for(i=1024; i>=0; i--) {
		analogWrite(PWM_PIN, i);
		analogWrite(PWM_PIN1, 1024-i);
		delay(10);
	}
}

void loop1()
{
	int test_pin;
	for(int i=0; i<pin_count; i++) {
		Serial.print("D");
		Serial.println(i);
		test_pin = pins[i];
		pinMode(test_pin, OUTPUT);
		for(int j=0; j<60; j++) {
			digitalWrite(test_pin, HIGH);
			delay(5);
			digitalWrite(test_pin, LOW);
			delay(5);
		}
		pinMode(test_pin, INPUT);
	}
}

void loop2()
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
//		delay(2000);
		Serial.print("pwm off:");
		Serial.println(i);
		sSer.print("PWMS");
		sSer.println(i);
//		delay(2000);
	}

}

