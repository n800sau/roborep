#include "../../include/common.h"
#include "../../include/printf.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <voltage.h>

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(8,10);

// Network uses that radio
RF24Network network(radio);

// How often to print voltage
const unsigned long interval = 5000; //ms

// When did we last print voltage?
unsigned long last_sent;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// ADDRESS: <address>
// line1
// line2
// line3
// ...
// :END

int current_address;
String cmdBuffer;

void setup(void)
{
	Serial.begin(57600);
	inputString.reserve(200);
	printf_begin();
	Serial.println("PC to nrf network interface");
 
	SPI.begin();
	radio.begin();
	network.begin(CHANNEL, PC2NRF_NODE);
	cmdBuffer.reserve(500);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void loop(void)
{
	// Pump the network regularly
	network.update();
	// print the string when a newline arrives:
	// check command from PC
	if (stringComplete) {
		if(current_address < 0) {
			// new command ???
			if(inputString.startsWith(ADDRESS_MARKER)) {
				current_address = inputString.substring(sizeof(ADDRESS_MARKER)).toInt();
			}
		} else {
			if(inputString.equals(END_MARKER)) {
				// send command buffer to the current address
				int payload_size = cmdBuffer.length() + 2;
				byte payload[payload_size];
				cmdBuffer.getBytes(payload, payload_size);
				RF24NetworkHeader header(current_address);
				if (network.write(header,&payload,sizeof(payload)))
					Serial.println("ok.");
				else
					Serial.println("failed.");
				current_address = -1;
				cmdBuffer = "";
			} else {
				cmdBuffer += inputString + '\n';
			}
		}
		// clear the string:
		inputString = "";
		stringComplete = false;
	}
	// check network
	if ( network.available() ) {
		RF24NetworkHeader header;
		payload_t reply;
		network.read(header,&reply,sizeof(reply));

		switch(reply.pload_type) {
			case PL_MPU:
				Serial.print(REPLY_MARKER);
				Serial.print(" #");
				Serial.print(reply.counter);
				Serial.print(" ms ");
				Serial.print(reply.ms);
				Serial.print("\tquat\t");
				Serial.print(reply.d.mpu.quaternion[0]);
				Serial.print("\t");
				Serial.print(reply.d.mpu.quaternion[1]);
				Serial.print("\t");
				Serial.print(reply.d.mpu.quaternion[2]);
				Serial.print("\t");
				Serial.print(reply.d.mpu.quaternion[3]);
				Serial.print("\tgrav\t");
				Serial.print(reply.d.mpu.gravity[0]);
				Serial.print("\t");
				Serial.print(reply.d.mpu.gravity[1]);
				Serial.print("\t");
				Serial.println(reply.d.mpu.gravity[2]);
				break;
			case PL_ACC: 
				Serial.print(REPLY_MARKER);
				Serial.print(" #");
				Serial.print(reply.counter);
				Serial.print(" ms ");
				Serial.print(reply.ms);
				Serial.print("\tacc\t");
				Serial.print(reply.d.acc.raw[0] * (int)reply.d.acc.uScale);
				Serial.print("\t");
				Serial.print(reply.d.acc.raw[1] * (int)reply.d.acc.uScale);
				Serial.print("\t");
				Serial.println(reply.d.acc.raw[2] * (int)reply.d.acc.uScale);
				break;
			default:
				Serial.print(REPLY_MARKER);
				Serial.print(" #");
				Serial.print(reply.counter);
				Serial.print(" ms ");
				Serial.println(reply.ms);
				break;
		}
	} else {
		// If it's time to send a message, send it!
		unsigned long now = millis();
		if ( now - last_sent >= interval)
		{
			last_sent = now;
			Serial.print(CONTROLLER_STATE_MARKER);
			Serial.print("\tv\t");
			Serial.println(readVccMv(), DEC);
		}
//		Serial.println("Nothing");
	}
	delay(10);
}

