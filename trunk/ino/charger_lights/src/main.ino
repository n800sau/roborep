#include <EventFuse.h>
#include <JsonParser.h>
#include "../../include/printf.h"
#include <IRremote.h>

#define IR_PIN 13

IRsend ir;


#define MAX_INPUT_LEN 200
char inputString[MAX_INPUT_LEN];
int inputPos = 0;
boolean stringComplete = false;

#define SIGNAL_LEN 6
static unsigned int irsignal[SIGNAL_LEN] = {10, 20, 10, 20, 10, 20};

void FuseEvent(FuseID fuse, int& led_index){
	ir.sendRaw(irsignal, SIGNAL_LEN, 56);
//	Serial.print("event:");
//	Serial.print(led_index);
//	Serial.print(":");
//	Serial.println(pin->state);
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
	EventFuse::newFuse(500, INF_REPEAT, FuseEvent);
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

	delayMicroseconds(100);
	EventFuse::burn();
}
