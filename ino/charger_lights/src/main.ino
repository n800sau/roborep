#include <EventFuse.h>
#include <JsonParser.h>
#include "../../include/printf.h"

struct LED {
	int pin;
	boolean state;
	int count;
};

#define N_LEDS 3

LED pins[N_LEDS] = {
	{13, LOW, 1000},
	{12, LOW, 2000},
	{11, LOW, 3000}
};

#define MAX_INPUT_LEN 200
char inputString[MAX_INPUT_LEN];
int inputPos = 0;
boolean stringComplete = false;

void FuseEvent(FuseID fuse, int& led_index){
	LED *pin = pins + led_index;
	pin->state = !pin->state;
	digitalWrite(pin->pin, pin->state);
	printf("{\"led\":%d,\"on\":%d}\r\n", led_index, pin->state);
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
	for(int i=0; i<N_LEDS; i++) {
		LED *pin = pins + i;
		pinMode(pin->pin, OUTPUT);
		EventFuse::newFuse(i, pin->count, INF_REPEAT, FuseEvent);
	}
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
