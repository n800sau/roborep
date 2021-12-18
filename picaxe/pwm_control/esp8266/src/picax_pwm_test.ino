// On ESP8266:
// At 80MHz runs up 57600ps, and at 160MHz CPU frequency up to 115200bps with only negligible errors.
// Connect pin 12 to 14.

#define SW_SERIAL_TX D8
// if Serial then pin TX of Serial

#include <SoftwareSerial.h>

#define BAUD_RATE 19200

// Reminder: the buffer size optimizations here, in particular the isrBufSize that only accommodates
// a single 8N1 word, are on the basis that any char written to the loopback SoftwareSerial adapter gets read
// before another write is performed. Block writes with a size greater than 1 would usually fail. 
SoftwareSerial swSer;

void setup() {
	Serial.begin(BAUD_RATE);
	swSer.begin(BAUD_RATE, SWSERIAL_8N1, -1, SW_SERIAL_TX);

	Serial.println("\nSoftware serial started");

}

//#define CommandSerial Serial
#define CommandSerial swSer

char cmdbuf[128];

void send_cmd(const char *cmd)
{
	for(size_t i=0;i<strlen(cmd); i++) {
		CommandSerial.write(cmd[i]);
		delay(5);
	}
}

void pwm_on(int pindex, int period, int duty)
{
	snprintf(cmdbuf, sizeof(cmdbuf), "XXP%d PER%d DUT%d ", pindex, period, duty);
	send_cmd(cmdbuf);
}

void pwm_off(int pindex)
{
	snprintf(cmdbuf, sizeof(cmdbuf), "XXS%d ", pindex);
	send_cmd(cmdbuf);
}

void pin_on(int pindex)
{
	snprintf(cmdbuf, sizeof(cmdbuf), "XXB%d L1 ", pindex);
	send_cmd(cmdbuf);
}

void pin_off(int pindex)
{
	snprintf(cmdbuf, sizeof(cmdbuf), "XXB%d L0 ", pindex);
	send_cmd(cmdbuf);
}

void loop()
{
	pwm_on(1, 100, 400);
		Serial.println("1 400");
			delay(1000);
	pwm_on(1, 100, 0);
		Serial.println("1 0");
			delay(1000);
	pwm_on(1, 100, 300);
		Serial.println("1 300");
			delay(1000);
	pwm_on(2, 100, 100);
		Serial.println("2 100");
			delay(1000);
	pwm_on(2, 100, 300);
		Serial.println("2 300");
			delay(1000);
	
/*
	for(int i=1; i<=2; i++) {
		Serial.print(i);
		Serial.println(" up");
		for(int j=10; j<400; j+=10) {
			pwm_on(i, 100, j);
			delay(10);
		}
		Serial.print(i);
		Serial.println(" down");
		for(int j=400; j>=0; j-=10) {
			pwm_on(i, 100, j);
			delay(10);
		}
		pwm_off(i);
		delay(1000);
	}
*/
/*
	for(int i=1; i<=2; i++) {
		pin_on(i);
		Serial.print(i);
		Serial.println(" up");
		delay(1000);
		pin_off(i);
		Serial.print(i);
		Serial.println(" down");
		delay(1000);
	}
*/
}
