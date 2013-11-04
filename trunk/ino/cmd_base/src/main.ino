#include "../../include/common.h"
#include "../../include/printf.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(8,10);

// Network uses that radio
RF24Network network(radio);

// How often to send 'hello world to the other unit
const unsigned long interval = 2000; //ms

// When did we last send?
unsigned long last_sent;

// How many have we sent already
unsigned long packets_sent;

void setup(void)
{
	Serial.begin(57600);
	printf_begin();
	Serial.println("Commander");
 
	SPI.begin();
	radio.begin();
	network.begin(CHANNEL, BASE_NODE);

}

bool has_reply = true;

void loop(void)
{
	// Pump the network regularly
	network.update();

	if(has_reply) {
		// If it's time to send a message, send it!
		unsigned long now = millis();
		if ( now - last_sent >= interval)
		{
			last_sent = now;
	
			payload_t payload = { PL_CMD, millis(), packets_sent++ };
			payload.d.cmd.cmd = CMD_MPU;

//			Serial.print("Sending...");
//			Serial.print(payload.counter);
//			Serial.print(", ms=");
//			Serial.print(payload.ms);
//			Serial.print(", ");
//			RF24NetworkHeader header(STICK_NODE);
//			bool ok = network.write(header,&payload,sizeof(payload));
//			if (ok)
//				Serial.println("ok.");
//			else
//				Serial.println("failed.");
			has_reply = false;
		}
	} else {
		if ( network.available() ) {
			RF24NetworkHeader header;
			payload_t payload;
			network.read(header,&payload,sizeof(payload));
			Serial.print("Received packet #");
			Serial.print(payload.counter);
			Serial.print(" at ");
			Serial.println(payload.ms);
			Serial.print("quat\t");
			Serial.print(payload.d.mpu.quaternion[0]);
			Serial.print("\t");
			Serial.print(payload.d.mpu.quaternion[1]);
			Serial.print("\t");
			Serial.print(payload.d.mpu.quaternion[2]);
			Serial.print("\t");
			Serial.println(payload.d.mpu.quaternion[3]);
			has_reply = true;
		}
	}
	delay(100);
}
