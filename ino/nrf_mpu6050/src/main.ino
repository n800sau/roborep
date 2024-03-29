#include "../../include/common.h"
#include "../../include/printf.h"
#include <avr/sleep.h>
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif
#include "MPU6050_6Axis_MotionApps20.h"


// nRF24L01(+) radio attached using Getting Started board 
RF24 radio(9,10);

// Network uses that radio
RF24Network network(radio);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
// ----------------------------------------------------------------------
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

const int interruptPin = 1;

volatile bool mpuInterrupt = false;		// indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
//	detachInterrupt(interruptPin);
}
// ----------------------------------------------------------------------

void setup()
{
	Serial.begin(57600);
	printf_begin();
	Serial.println("NRF_MPU6050");
 
	SPI.begin();
	radio.begin();
	network.begin(CHANNEL, NRF_MPU6050_NODE);

	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	Serial.println(PSTR("Initializing MPU..."));
	mpu.initialize();

	// set interrupt active to low
	mpu.setInterruptMode(MPU6050_INTMODE_ACTIVELOW);

//mpu.setMotionDetectionThreshold();
//mpu.setZeroMotionDetectionThreshold();
//mpu.setMotionDetectionDuration();
//mpu.setZeroMotionDetectionDuration();
//mpu.setIntEnabled();
//mpu.setSleepEnabled();
//mpu.setWakeCycleEnabled();


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
		attachInterrupt(interruptPin, dmpDataReady, FALLING);
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

unsigned long ms = 0;
unsigned long pcounter = 0;


//pause between value output (ms)
const int printDelay = 5000;

const float Q_THRESHOLD = 0.002;

Quaternion last_q;

void read_cmd()
{
	// If so, grab it and print it out
	RF24NetworkHeader header;
	payload_t payload;
	network.read(header,&payload,sizeof(payload));
	Serial.print("Received packet #");
	Serial.print(payload.counter);
	Serial.print(" at ");
	Serial.println(payload.ms);
	Serial.print("TYPE:");
	Serial.println(payload.pload_type);
	switch(payload.pload_type) {
		default:
			break;
	}
}

void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we 
     * choose the according 
     * sleep mode: SLEEP_MODE_PWR_DOWN
     * 
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    /* Now it is time to enable an interrupt. We do it here so an 
     * accidentally pushed interrupt button doesn't interrupt 
     * our running program. if you want to be able to run 
     * interrupt code besides the sleep function, place it in 
     * setup() for example.
     * 
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
     * 
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */

    attachInterrupt(interruptPin, dmpDataReady, LOW); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW 

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(interruptPin);      // disables interrupt 0 on pin 2 so the 
                             // wakeUpNow code will not be executed 
                             // during normal running time.

}

void loop()
{
	unsigned long qms;
//	delay(100);                           // waits for a second
//	if (mpuInterrupt || fifoCount >= packetSize) {
	if (mpuInterrupt) {
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

			Quaternion q;
			mpu.dmpGetQuaternion(&q, fifoBuffer);
//			Serial.print("Difference:");
//			Serial.println(q.w);

			// send if timeout or acceleration change is bigger that threshold
			if(abs(millis() - ms) > printDelay || abs(q.w - last_q.w) > Q_THRESHOLD) {
				last_q = q;

				payload_t reply;
				mpu.dmpGetQuaternion(reply.d.mpu.quaternion, fifoBuffer);
				VectorFloat gravity;
				mpu.dmpGetGravity(&gravity, &q);
				reply.d.mpu.gravity[0] = gravity.x * 100;
				reply.d.mpu.gravity[1] = gravity.y * 100;
				reply.d.mpu.gravity[2] = gravity.z * 100;
				// display quaternion values in easy matrix form: w x y z
				RF24NetworkHeader header(PC2NRF_NODE);
				qms = millis();
				reply.pload_type = PL_MPU;
				reply.ms = qms;
				reply.counter = ++pcounter;

/*				Serial.print("Sending...");
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
				Serial.print("grav\t");
				Serial.print(reply.d.mpu.gravity[0]);
				Serial.print("\t");
				Serial.print(reply.d.mpu.gravity[1]);
				Serial.print("\t");
				Serial.println(reply.d.mpu.gravity[2]);
*/
				bool ok = network.write(header,&reply,sizeof(reply));
				if (ok)
					Serial.println("Replied ok.");
				else
					Serial.println("Reply failed.");
				radio.powerDown();
				ms = millis();
			}
//			mpu.dmpGetEuler(data.euler, &data.q);
//			mpu.dmpGetGravity(&data.gravity, &data.q);
//			mpu.dmpGetYawPitchRoll(data.ypr, &data.q, &data.gravity);
//			mpu.dmpGetAccel(&data.aa, fifoBuffer);
//			mpu.dmpGetLinearAccel(&data.aaReal, &data.aa, &data.gravity);
//			mpu.dmpGetLinearAccelInWorld(&data.aaWorld, &data.aaReal, &data.q);
		}
//		attachInterrupt(interruptPin, dmpDataReady, FALLING);
	}
	// Pump the network regularly
	network.update();
	// Is there anything ready for us?
	while ( network.available() )
	{
		Serial.println("read cmd");
		read_cmd();
	}
    delay(100);     // this delay is needed, the sleep 
    sleepNow();     // sleep function called here
}
