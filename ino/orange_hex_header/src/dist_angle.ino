// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "MPU6050_6Axis_MotionApps20.h"
#include <VL53L0X.h>
#include <EventFuse.h>
#include "const.h"
#include "servos.h"

VL53L0X sensor;

enum MODE {MODE_IDLE, MODE_SWING, MODE_GIVE};
MODE mode = MODE_IDLE;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 mpu;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	Serial.begin(115200);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	Serial.println(F("\ndist_angle start"));

	setup_servos();

	mpu.initialize();
	pinMode(INTERRUPT_PIN2, INPUT);
	pinMode(INTERRUPT_PIN3, INPUT);
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
//	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), dmpDataReady, RISING);
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN3), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		Serial.print(F("Packet size: "));
		Serial.println(packetSize);

		sensor.init();
		sensor.setTimeout(50);
		// lower the return signal rate limit (default is 0.25 MCPS (Millions of Cycles Per Second))
//		sensor.setSignalRateLimit(0.1);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
//		sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//		sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	delay(500);
	digitalWrite(LED_PIN, LOW);
	mpu.resetFIFO();
}

namespace Swing
{
	const int pan_step_ticks = 20;
	int tilt;
	int pan;
	int pan_dir;

	void pan_next(FuseID fuse, int& userData);

	void tilt_incr(FuseID fuse, int& userData)
	{
		tilt += SONAR_TILT_INCR;
		head_tilt_servo.write(tilt);
//		Serial.print(F(" tilt="));
//		Serial.println(tilt);
		if(tilt < SONAR_TILT_ANGLE_MAX) {
			EventFuse::newFuse(pan_step_ticks, 1, pan_next);
		} else {
			Serial.println(F("End collecting"));
			center_servos();
			delay(500);
			detach_servos();
			mode = MODE_IDLE;
		}
	}

	void pan_next(FuseID fuse, int& userData)
	{
		pan += pan_dir;
		if(pan_dir > 0 && pan > SONAR_PAN_ANGLE_MAX) {
			pan_dir = -1;
			EventFuse::newFuse(pan_step_ticks, 1, tilt_incr);
		} else if(pan_dir < 0 && pan < SONAR_PAN_ANGLE_MIN) {
			pan_dir = 1;
			EventFuse::newFuse(pan_step_ticks, 1, tilt_incr);
		} else {
			head_pan_servo.write(pan);
			EventFuse::newFuse(pan_step_ticks, 1, pan_next);
		}
	}

	void run()
	{
		center_servos();
		delay(1000);
		Serial.println("Start collecting");
		tilt = SONAR_TILT_ANGLE_MIN;
		head_tilt_servo.write(tilt);
		pan = SONAR_PAN_ANGLE_MIN;
		pan_dir = 1;
		head_pan_servo.write(pan);
		mpu.resetFIFO();
		EventFuse::newFuse(pan_step_ticks, 1, pan_next);
	}

}

void collect_data()
{
	bool IMUready = false;
	int pitch;
	int yaw;
	// wait for MPU interrupt or extra packet(s) available
	// if programming failed, don't try to do anything
	if (dmpReady) {
		if(mpuInterrupt)
		{
			digitalWrite(LED_PIN, LOW);
			// reset interrupt flag and get INT_STATUS byte
			mpuInterrupt = false;
			mpuIntStatus = mpu.getIntStatus();

			// get current FIFO count
			fifoCount = mpu.getFIFOCount();

			// check for overflow (this should never happen unless our code is too inefficient)
			if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {

				if(mode == MODE_SWING) {
					// reset so we can continue cleanly
					Serial.print(millis());
					Serial.print(F(" FIFO overflow: "));
					Serial.println(fifoCount);
				}
				mpu.resetFIFO();

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
			} else if (mpuIntStatus & 0x02) {
				// wait for correct available data length, should be a VERY short wait
				while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

				while(fifoCount >= packetSize) {
					// read a packet from FIFO
					mpu.getFIFOBytes(fifoBuffer, packetSize);

					// track FIFO count here in case there is > 1 packet available
					// (this lets us immediately read more without waiting for an interrupt)
					fifoCount -= packetSize;
				}
//				Serial.print(F("Fifo reduced to "));
//				Serial.println(fifoCount);

				// display Euler angles in degrees
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				pitch = ypr[1] * 180/M_PI;
				yaw = ypr[0] * 180/M_PI;
				IMUready = true;
			}
		}
	} 
	if(IMUready)
	{
		int df;
		int mm = sensor.readRangeSingleMillimeters();
		if (sensor.timeoutOccurred()) {
//			Serial.print(F("TIMEOUT"));
			mm = -1;
		}
		switch(mode) {
			case MODE_SWING:
				df = Swing::pan - yaw;
				if(df > 180) {
					df = 360 - df;
				}
				Serial.print("!d#,");
				Serial.print("t,");
				Serial.print(Swing::tilt);
				Serial.print("p,");
				Serial.print(Swing::pan);
				Serial.print(",y,");
				Serial.print(yaw);
				Serial.print(",-,");
				Serial.print(df);
				Serial.print(",p,");
				Serial.print(pitch);
				Serial.print(",m,");
				Serial.println(mm);
				break;
			case MODE_IDLE:
//				Serial.print(F("mm="));
//				Serial.print(mm);
//				Serial.print(F(", yaw="));
//				Serial.print(yaw);
//				Serial.print(F(", pitch="));
//				Serial.println(pitch);
				break;
			default:
				break;
		}
	} else {
		digitalWrite(LED_PIN, HIGH);
	}
}

unsigned long last_ms = millis();

void loop()
{
	unsigned long ms;
	if(Serial.available()) {
		switch(Serial.read()) {
			case 'r':
				if(mode == MODE_IDLE) {
					mode = MODE_SWING;
					Swing::run();
				} else {
					Serial.println(F("Can not start swing. Mode is not idle."));
				}
				break;
		}
	}
	collect_data();
	ms = last_ms;
	last_ms = millis();
	if(last_ms - ms > 1) {
		if(mode == MODE_SWING) {
//			Serial.print("Burn ");
//			Serial.println(last_ms - ms);
		}
		EventFuse::burn(last_ms - ms);
	}
}

