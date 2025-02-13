// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#include <IRremote.h>

#define RECV_PIN 4

IRrecv irrecv(RECV_PIN);

decode_results results;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

#define OUTPUT_RAW_ACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector


// ================================================================
// ===				 INTERRUPT DETECTION ROUTINE				===
// ================================================================

volatile bool mpuInterrupt = false;		// indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}



// ================================================================
// ===						INITIAL SETUP						===
// ================================================================

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(57600);
//	  while (!Serial); // wait for Leonardo enumeration, others continue immediately

	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// wait for ready
//	  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//	  while (Serial.available() && Serial.read()); // empty buffer
//	  while (!Serial.available());				   // wait for data
//	  while (Serial.available() && Serial.read()); // empty buffer again

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	enum CORR_TYPE {AX=0, AY, AZ, GX, GY, GZ};
	int16_t corrs[] = {-5130, 1320, -1017, -141, 55, 86};
	mpu.setXAccelOffset(corrs[AX]);
	mpu.setYAccelOffset(corrs[AY]);
	mpu.setZAccelOffset(corrs[AZ]);
	mpu.setXGyroOffset(corrs[GX]);
	mpu.setYGyroOffset(corrs[GY]);
	mpu.setZGyroOffset(corrs[GZ]);

    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);

	irrecv.enableIRIn(); // Start the receiver

}

void dump(decode_results *results) 
{
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
//	  Serial.print("Unknown encoding: ");
  } else {
  if (results->decode_type == NEC) {
	Serial.print("Decoded NEC: ");
  } 
  else if (results->decode_type == SONY) {
	Serial.print("Decoded SONY: ");
  } 
  else if (results->decode_type == RC5) {
	Serial.print("Decoded RC5: ");
  } 
  else if (results->decode_type == RC6) {
	Serial.print("Decoded RC6: ");
  }
  else if (results->decode_type == PANASONIC) {	
	Serial.print("Decoded PANASONIC - Address: ");
	Serial.print(results->panasonicAddress,HEX);
	Serial.print(" Value: ");
  }
  else if (results->decode_type == JVC) {
	 Serial.print("Decoded JVC: ");
  }
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  for (int i = 0; i < count; i++) {
	if ((i % 2) == 1) {
	  Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
	} 
	else {
	  Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
	}
	Serial.print(" ");
  }
  Serial.println("");
  }
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

int send_power_millis = 0;

// ================================================================
// ===					  MAIN PROGRAM LOOP						===
// ================================================================

void loop() {

	if(send_power_millis + 1000 < millis()) {
		Serial.print("vcc\t");
		Serial.println(readVcc());
		send_power_millis = millis();
	}

	if (irrecv.decode(&results)) {
		dump(&results);
		if(results.decode_type == NEC) {
			Serial.print("ir\t");
			Serial.print(results.value);
			Serial.println();
		}
		irrecv.resume(); // Receive the next value
	}


	// if programming failed, don't try to do anything
	if (dmpReady) {

		// wait for MPU interrupt
		if (mpuInterrupt) {

			// reset interrupt flag and get INT_STATUS byte
			mpuInterrupt = false;
			mpuIntStatus = mpu.getIntStatus();

			// check for overflow (this should never happen unless our code is too inefficient)
			if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
				// reset so we can continue cleanly
				mpu.resetFIFO();
				Serial.println(F("error\tFIFO overflow!"));

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
			} else if (mpuIntStatus & 0x02) {

				// get current FIFO count
				fifoCount = mpu.getFIFOCount();

				while(fifoCount >= packetSize) {
    
					// read a packet from FIFO
					mpu.getFIFOBytes(fifoBuffer, packetSize);
    
					// track FIFO count here in case there is > 1 packet available
					// (this lets us immediately read more without waiting for an interrupt)
					fifoCount -= packetSize;
 
					Serial.println(F("START"));

					#ifdef OUTPUT_READABLE_QUATERNION
						// display quaternion values in easy matrix form: w x y z
						mpu.dmpGetQuaternion(&q, fifoBuffer);
						Serial.print("quat\t");
						Serial.print(q.w);
						Serial.print("\t");
						Serial.print(q.x);
						Serial.print("\t");
						Serial.print(q.y);
						Serial.print("\t");
						Serial.println(q.z);
					#endif
        
					#ifdef OUTPUT_READABLE_EULER
						// display Euler angles in radians
						mpu.dmpGetQuaternion(&q, fifoBuffer);
						mpu.dmpGetEuler(euler, &q);
						Serial.print("euler\t");
						Serial.print(euler[0]);
						Serial.print("\t");
						Serial.print(euler[1]);
						Serial.print("\t");
						Serial.println(euler[2]);
					#endif
        
					#ifdef OUTPUT_READABLE_YAWPITCHROLL
						// display Euler angles in radians
						mpu.dmpGetQuaternion(&q, fifoBuffer);
						mpu.dmpGetGravity(&gravity, &q);
						mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
						Serial.print("ypr\t");
						Serial.print(ypr[0]);
						Serial.print("\t");
						Serial.print(ypr[1]);
						Serial.print("\t");
						Serial.println(ypr[2]);
					#endif
        
					#ifdef OUTPUT_RAW_ACCEL
						// display real acceleration, adjusted to remove gravity
						mpu.dmpGetAccel(&aa, fifoBuffer);
						Serial.print("acc\t");
						Serial.print(aa.x);
						Serial.print("\t");
						Serial.print(aa.y);
						Serial.print("\t");
						Serial.println(aa.z);
					#endif
        
					#ifdef OUTPUT_READABLE_REALACCEL
						// display real acceleration, adjusted to remove gravity
						mpu.dmpGetQuaternion(&q, fifoBuffer);
						mpu.dmpGetAccel(&aa, fifoBuffer);
						mpu.dmpGetGravity(&gravity, &q);
						mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
						Serial.print("areal\t");
						Serial.print(aaReal.x);
						Serial.print("\t");
						Serial.print(aaReal.y);
						Serial.print("\t");
						Serial.println(aaReal.z);
					#endif
        
					#ifdef OUTPUT_READABLE_WORLDACCEL
						// display initial world-frame acceleration, adjusted to remove gravity
						// and rotated based on known orientation from quaternion
						mpu.dmpGetQuaternion(&q, fifoBuffer);
						mpu.dmpGetAccel(&aa, fifoBuffer);
						mpu.dmpGetGravity(&gravity, &q);
						mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
						mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
						Serial.print("aworld\t");
						Serial.print(aaWorld.x);
						Serial.print("\t");
						Serial.print(aaWorld.y);
						Serial.print("\t");
						Serial.println(aaWorld.z);
					#endif
					Serial.println(F("END"));
					// blink LED to indicate mpu activity
					blinkState = !blinkState;
					digitalWrite(LED_PIN, blinkState);
				}
			}
		}

	}
//	delay(10);
}

