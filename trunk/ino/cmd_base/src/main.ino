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

//	set_servo(90, 90, 90);
}

bool set_servo(uint8_t low, uint8_t pan, uint8_t tilt)
{
	payload_t payload = { PL_SETSERVO, millis(), packets_sent++ };
	payload.d.servo.low = low;
	payload.d.servo.pan = pan;
	payload.d.servo.tilt = tilt;
	Serial.print("Sending...");
//	Serial.print(payload.counter);
//	Serial.print(", ms=");
//	Serial.print(payload.ms);
//	Serial.print(", ");
	Serial.print(payload.d.servo.low);
	Serial.print("\t");
	Serial.print(payload.d.servo.pan);
	Serial.print("\t");
	Serial.println(payload.d.servo.tilt);
	RF24NetworkHeader header(STICK_NODE);
	bool rs = network.write(header,&payload,sizeof(payload));
	Serial.println("Sent");
	return rs;
}

#define STEP 1
int tiltdir = STEP;
stickservo_t servo = {90, 90, 90};

void move_servo()
{
	int pos = servo.pan + tiltdir;
	if(pos >= 180) {
		tiltdir = -STEP;
	}
	if(pos <= 0) {
		tiltdir = STEP;
	}
	if (set_servo(NO_SET, pos, NO_SET))
		Serial.println("ok.");
	else
		Serial.println("failed.");
}

void loop(void)
{
	// Pump the network regularly
	network.update();
	if( Serial.available() ) {
		if(Serial.find(CMD_MARKER)) {
			char buf[50];
			// read command
			int len = Serial.readBytesUntil(' ', buf, sizeof(buf)-1);
			buf[len] = 0;
			if(len > 0) {
				if( String("reset").equalsIgnoreCase(String(buf)) ) {
					stickservo_t ss = {90, 90, 90};
					servo = ss;
					Serial.print(REPLY_MARKER);
					Serial.println("Reset to 90,90,90");
				} else if ( String("low_pos").equalsIgnoreCase(String(buf)) ) {
					len = Serial.readBytesUntil(' ', buf, sizeof(buf)-1);
					buf[len] = 0;
					if(len > 0) {
						int pos = atoi(buf);
						if(pos > 0) {
							servo.low = pos;
							set_servo(servo.low, NO_SET, NO_SET);
						}
					}
					Serial.print(REPLY_MARKER);
					Serial.print("Low Servo at ");
					Serial.println(servo.low);
				}
			}
			Serial.flush();
		}
	}
	if ( network.available() ) {
		RF24NetworkHeader header;
		payload_t payload;
		network.read(header,&payload,sizeof(payload));
//		Serial.print("Received packet #");
//		Serial.print(payload.counter);
//		Serial.print(" at ");
//		Serial.println(payload.ms);
		switch(payload.pload_type) {
			case PL_MPU:
				Serial.print("quat\t");
				Serial.print(payload.d.mpu.quaternion[0]);
				Serial.print("\t");
				Serial.print(payload.d.mpu.quaternion[1]);
				Serial.print("\t");
				Serial.print(payload.d.mpu.quaternion[2]);
				Serial.print("\t");
				Serial.println(payload.d.mpu.quaternion[3]);
				Serial.print("grav\t");
				Serial.print(payload.d.mpu.gravity[0]);
				Serial.print("\t");
				Serial.print(payload.d.mpu.gravity[1]);
				Serial.print("\t");
				Serial.println(payload.d.mpu.gravity[2]);
				break;
			case PL_ACC:
				Serial.print("Accel\t");
				Serial.print(payload.d.acc.raw[0]);
				Serial.print("\t");
				Serial.print(payload.d.acc.raw[1]);
				Serial.print("\t");
				Serial.println(payload.d.acc.raw[2]);
				break;
			case PL_SERVO_STATE:
				Serial.print("received servo\t");
				Serial.print(payload.d.servo.low);
				Serial.print("\t");
				Serial.print(payload.d.servo.pan);
				Serial.print("\t");
				Serial.println(payload.d.servo.tilt);
				servo = payload.d.servo;
				break;
		}
	} else {
		// If it's time to send a message, send it!
		unsigned long now = millis();
		if ( now - last_sent >= interval)
		{
			last_sent = now;
//			move_servo();
		}
//		Serial.println("Nothing");
	}
	delay(10);
}
