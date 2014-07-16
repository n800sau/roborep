#include <EventFuse.h>
#include <JsonParser.h>
#include "../../include/printf.h"
#include "../../include/common.h"

#include <IRremote.h>


#define LED_PIN 13

IRsend ir;


#define MAX_INPUT_LEN 200
char inputString[MAX_INPUT_LEN];
int inputPos = 0;
boolean stringComplete = false;

#define SIGNAL_LEN 4
static unsigned int irsignal[SIGNAL_LEN] = {10, 20, 30, 40};

void FuseEvent(FuseID fuse, int& led_index){
//	ir.sendRaw(irsignal, SIGNAL_LEN, 56);
	ir.sendNEC(BEACON_CODE, 32);
//	ir.sendSony(0x00000826, 15);
//	ir.sendDISH(BEACON_CODE, 32);
	digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//	Serial.println("NEC");
}

void serialEvent() {
  while (Serial.available()) {
	char inChar = (char)Serial.read();
	// add it to the inputString:
	inputString[inputPos++] = inChar;
	inputString[inputPos] = 0;
	if (inChar == '\n' || inputPos >= MAX_INPUT_LEN-1) {
	  stringComplete = true;
	}
  }
}

void setup() {
	Serial.begin(57600);
	printf_begin();
	EventFuse::newFuse(2000, INF_REPEAT, FuseEvent);
//	Serial.println("setup");
}

void loop(){
	if (stringComplete) {
		JsonParser<32> parser;
		JsonHashTable data = parser.parseHashTable(inputString);
		char* name = data.getString("command");
		printf("Executing %s...\n", name);
		inputString[0] = 0;
		stringComplete = false;
	}

	delayMicroseconds(1000);
	EventFuse::burn();
}
