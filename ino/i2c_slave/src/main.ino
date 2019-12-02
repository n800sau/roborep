#include <Wire.h>

#define I2C_ADDR 0x68

char cur_reg = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println("\nI2C Slave");

	Wire.begin(I2C_ADDR);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

}

volatile int received = 0;
volatile int requested = 0;

void loop()
{
	if(received) {
		Serial.print("Received ");
		Serial.println(received);
		received = 0;
	}
	if(requested) {
		Serial.print("Request for ");
		Serial.println(cur_reg, HEX);
		requested = 0;
	}
//	delay(2);
}

void receiveEvent(int count)
{
	received = count;
	for(int i=0; i<count; i++) {
		cur_reg = Wire.read();
	}
}

void requestEvent()
{
	requested = 1;
	switch(cur_reg) {
		case 0:
			Wire.write(0xe5);
			break;
		case 'A':
			Wire.write(cur_reg + 1);
			break;
		default:
			Wire.write(-1);
			break;
	}
}
