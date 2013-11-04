#include "../../include/common.h"
#include "../../include/printf.h"
#include <Servo.h>
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif
#include "MPU6050_6Axis_MotionApps20.h"


// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(8,10);

// Network uses that radio
RF24Network network(radio);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

Servo lowservo;
Servo tiltservo;
Servo panservo;


// MPU control/status vars
// ----------------------------------------------------------------------
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;		// indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}
// ----------------------------------------------------------------------

void setup()
{
	Serial.begin(57600);
	printf_begin();
	Serial.println("Stick");
 
	SPI.begin();
	radio.begin();
	network.begin(CHANNEL, STICK_NODE);

	//init positions
	lowservo.attach(9);
	tiltservo.attach(6);
	panservo.attach(5);
	lowservo.write(90);
	tiltservo.write(90);
	panservo.write(90);

	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	Serial.println(PSTR("Initializing MPU..."));
	mpu.initialize();

	// load and configure the DMP
	Serial.println(PSTR("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println("Enabling DMP...");
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println("DMP ready! Waiting for first interrupt...");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print("DMP Initialization failed (code");
		Serial.print(devStatus);
		Serial.println(")");
	}
}

int tiltdir = 1;
unsigned long ms = 0;
unsigned long pcounter = 0;

void loop()
{
	int pos = tiltservo.read() + tiltdir;
	if(pos >= 180) {
		tiltdir = -1;
	}
	if(pos <= 0) {
		tiltdir = 1;
	}
	tiltservo.write(pos);
//	Quaternion q;
	unsigned long qms;
	while (!mpuInterrupt && fifoCount < packetSize) {
		// Pump the network regularly
		network.update();
		// Is there anything ready for us?
		while ( network.available() )
		{
			// If so, grab it and print it out
			RF24NetworkHeader header;
			payload_t payload;
			network.read(header,&payload,sizeof(payload));
			Serial.print("Received packet #");
			Serial.print(payload.counter);
			Serial.print(" at ");
			Serial.println(payload.ms);
			Serial.print("CMD:");
			Serial.println(payload.d.cmd.cmd);
			switch(payload.d.cmd.cmd) {
				case CMD_MPU:
					break;
			}
		}
	}
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println("FIFO overflow!");

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) {
			fifoCount = mpu.getFIFOCount();
		}

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		payload_t reply;
		mpu.dmpGetQuaternion(reply.d.mpu.quaternion, fifoBuffer);
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
			// display quaternion values in easy matrix form: w x y z
		RF24NetworkHeader header(BASE_NODE);
		if(millis() - ms > 5000) {
			qms = millis();
			reply.pload_type = PL_MPU;
			reply.ms = qms;
			reply.counter = ++pcounter;
			Serial.print("Sending...");
			Serial.print(reply.counter);
			Serial.print(", ms=");
			Serial.println(reply.ms);
			Serial.print("quat\t");
			Serial.print(reply.d.mpu.quaternion[0]);
			Serial.print("\t");
			Serial.print(reply.d.mpu.quaternion[1]);
			Serial.print("\t");
			Serial.print(reply.d.mpu.quaternion[2]);
			Serial.print("\t");
			Serial.println(reply.d.mpu.quaternion[3]);
			bool ok = network.write(header,&reply,sizeof(reply));
			if (ok)
				Serial.println("Replied ok.");
			else
				Serial.println("Reply failed.");
			ms = millis();
		}
//		mpu.dmpGetEuler(data.euler, &data.q);
//		mpu.dmpGetGravity(&data.gravity, &data.q);
//		mpu.dmpGetYawPitchRoll(data.ypr, &data.q, &data.gravity);
//		mpu.dmpGetAccel(&data.aa, fifoBuffer);
///		mpu.dmpGetLinearAccel(&data.aaReal, &data.aa, &data.gravity);
//		mpu.dmpGetLinearAccelInWorld(&data.aaWorld, &data.aaReal, &data.q);
	}
}
