#include "Adafruit_NECremote.h"
#include "SendOnlySoftwareSerial.h"
 
// Connect a 38KHz remote control sensor to the pin below
#define IRpin         11

uint16_t ir_code[] = {
	0x201,
	0x200,
	0x205,
	0x206,
	0x207,
	0x209,
	0x20A,
	0x20B,
	0x20D,
	0x20E,
	0x20F,
	0x212,
	0x211,
	0x213,
	0x217,
	0x303,
	0x210,
	0x21e,
	0x20C,
	0x21F,
	0x214,
	0x302,
	0x208,
	0x219,
	0x218,
	0x21B,
	0x21A,
	0x21D,
	0x21C,
	0x293,
	0x301,
	0x300,
	0x204,
	0x215
};


Adafruit_NECremote remote(IRpin);
SendOnlySoftwareSerial ss(9);

void setup(void) {
	Serial.begin(115200);
	ss.begin(9600);
	Serial.println("Ready to decode IR!");
}

int lastcode = -1;

const uint8_t CMD_BIAS = 1;

void send_code(uint16_t code)
{
	uint8_t ss_code = 0x80 | (code & 0xff);
	Serial.print(F("Received code #")); 
	Serial.print(code, HEX);
	for(uint8_t i=0; i<sizeof(ir_code) / sizeof(ir_code[0]); i++) {
		if(ir_code[i] == code) {
			ss_code = i + CMD_BIAS;
			break;
		}
	}
	ss.write(ss_code);
	Serial.print(F(", Send code #")); 
	Serial.println(ss_code, HEX);
}

void loop(void)
{
//	int c = remote.listen(200);  // milliseconds to wait before timing out!
	int c = remote.listen();  // Without a #, it means wait forever
	if (c >= 0) {
		lastcode = c;
		send_code(lastcode);
	} else if (c == -3) {
		// bug: it happends once after each code
//		Serial.print("Repeat (");
//		Serial.print(lastcode, HEX);
//		Serial.println(")");
//		send_code(lastcode);
	} else if (c == -2) {
		Serial.println("Data error");
	} else {
//		Serial.print(".");
		Serial.println("Timed out waiting!");
	}
}
