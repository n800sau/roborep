#include "../../include/common.h"
#include "../../include/printf.h"
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <Wire.h>
#include <ADXL345.h>

// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(8,10);

// Network uses that radio
RF24Network network(radio);

#define ACC_ADDRESS (0xA7>>1)
ADXL345 accel;

// Set up a pin we are going to use to indicate our status using an LED.
int accStatusPin = 7; // I'm using digital pin 2.

// packet counter
unsigned long pcounter = 0;

void setup(void)
{
	Serial.begin(57600);
	printf_begin();
	Serial.println("Gravity Sensor");
 
	SPI.begin();
	radio.begin();
	network.begin(CHANNEL, ACC_NODE);

	// Start the I2C Wire library so we can use I2C to talk to the accelerometer.
	Wire.begin();
	// Ready an LED to indicate our status.
	pinMode(accStatusPin, OUTPUT);
	accel = ADXL345(ACC_ADDRESS);
	connect2();
}

void connect2()
{
		if(accel.EnsureConnected()) {
			Serial.println("Connected to ADXL345.");
			digitalWrite(accStatusPin, HIGH); // If we are connected, light our status LED.
		} else {
			Serial.println("Could not connect to ADXL345.");
			digitalWrite(accStatusPin, LOW); // If we are not connected, turn our LED off.
		}
		// Set the range of the accelerometer to a maximum of 2G.
		accel.SetRange(2, true);
		// Tell the accelerometer to start taking measurements.
		accel.EnableMeasurements();
}

// Output the data down the serial port.
void Output(AccelerometerRaw raw, AccelerometerScaled scaled)
{
	payload_t reply;
	reply.d.acc.raw[0] = raw.XAxis;
	reply.d.acc.raw[1] = raw.YAxis;
	reply.d.acc.raw[2] = raw.ZAxis;
	reply.d.acc.uScale = accel.getUScale();
	RF24NetworkHeader header(BASE_NODE);
	reply.pload_type = PL_ACC;
	reply.ms = millis();
	reply.counter = ++pcounter;
	Serial.print("Sending...");
	Serial.print(reply.counter);
	Serial.print(", ms=");
	Serial.println(reply.ms);
	Serial.print("raw\t");
	Serial.print(reply.d.acc.raw[0]);
	Serial.print("\t");
	Serial.print(reply.d.acc.raw[1]);
	Serial.print("\t");
	Serial.print(reply.d.acc.raw[2]);
	Serial.print("\t");

	// Tell us about the this data, but scale it into useful units (G).
//	Serial.print("	\tScaled:\t");
//	Serial.print(scaled.XAxis);
//	Serial.print("G	 ");   
//	Serial.print(scaled.YAxis);
//	Serial.print("G	 ");   
//	Serial.print(scaled.ZAxis);
//	Serial.println("G");
	Serial.println("");

	bool ok = network.write(header,&reply,sizeof(reply));
	if (ok)
		Serial.println("Replied ok.");
	else
		Serial.println("Reply failed.");
}


void loop(void)
{
	// Pump the network regularly
	network.update();
	if(accel.IsConnected) // If we are connected to the accelerometer.
	{
		// Read the raw data from the accelerometer.
		AccelerometerRaw raw = accel.ReadRawAxis();
		//This data can be accessed like so:
		int xAxisRawData = raw.XAxis;
		
		// Read the *scaled* data from the accelerometer (this does it's own read from the accelerometer
		// so you don't have to ReadRawAxis before you use this method).
		// This useful method gives you the value in G thanks to the Love Electronics library.
		AccelerometerScaled scaled = accel.ReadScaledAxis();
		// This data can be accessed like so:
		float xAxisGs = scaled.XAxis;
		
		// We output our received data.
		Output(raw, scaled);
	} else {
		connect2();
	}
	delay(500);
}
