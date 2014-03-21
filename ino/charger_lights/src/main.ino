#include <EventFuse.h>


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

void FuseEvent(FuseID fuse, int& led_index){
	LED *pin = pins + led_index;
	pin->state = !pin->state;
	digitalWrite(pin->pin, pin->state);
//	Serial.print("event:");
//	Serial.print(led_index);
//	Serial.print(":");
//	Serial.println(pin->state);
}

void setup() {
//	Serial.begin(57600);
	for(int i=0; i<N_LEDS; i++) {
		LED *pin = pins + i;
		pinMode(pin->pin, OUTPUT);
		EventFuse::newFuse(i, pin->count, INF_REPEAT, FuseEvent);
	}
//	Serial.println("setup");
}

void loop(){
	delayMicroseconds(100);
	EventFuse::burn();
}
