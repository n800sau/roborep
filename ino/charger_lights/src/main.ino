#include <ros.h>
#include <charger_appeal/lighthouse_cmd.h>
#include <EventFuse.h>

ros::NodeHandle nh;
charger_appeal::lighthouse_cmd cmd_msg;

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

void messageCb(const charger_appeal::lighthouse_cmd& command_msg) {
	Serial.print(command_msg.command);
	Serial.print(",");
	Serial.println(command_msg.param1);
}

ros::Subscriber<charger_appeal::lighthouse_cmd> request("/charger_appeal/lighthouse_cmd", messageCb );

void setup() {
//	Serial.begin(57600);
	for(int i=0; i<N_LEDS; i++) {
		LED *pin = pins + i;
		pinMode(pin->pin, OUTPUT);
		EventFuse::newFuse(i, pin->count, INF_REPEAT, FuseEvent);
	}
//	Serial.println("setup");
	nh.initNode();
	nh.subscribe(request);
}

void loop(){
	nh.spinOnce();
	delayMicroseconds(100);
	EventFuse::burn();
}
