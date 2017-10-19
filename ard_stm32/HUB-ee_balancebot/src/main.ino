#include <PID_v1.h>
#include <Cmd.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HardWire.h"
//#include "Wire.h"
#include <HUBeeBMDWheel.h>
//#include <libmaple/iwdg.h>


HardWire HWire(2, I2C_FAST_MODE); // I2c1
//HardWire HWire(2, I2C_DUTY_16_9); // I2c1

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

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN PA15  // use pin 2 on Arduino Uno & most boards
#define LED_PIN PC13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;	  // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;	 // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		   // [w, x, y, z]		 quaternion container
VectorInt16 aa;		 // [x, y, z]			accel sensor measurements
VectorInt16 aaReal;	 // [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];		 // [psi, theta, phi]	Euler angle container
float ypr[3];		   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;	 // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
	mpuInterrupt = true;
}

//PID
const double originalSetpoint = 173;
double setpoint = originalSetpoint;
double input, output;

//adjust these values to fit your own design
//double Kp = 50;
double Kp = 50;
//double Kd = 1.4;
double Kd = 0;
//double Ki = 60;
double Ki = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// HUB-ee wheels

HUBeeBMDWheel motor1Wheel;
HUBeeBMDWheel motor2Wheel;

int motor1DirectionForward = 1;
int motor2DirectionForward = 0;
int motor1DirectionReverse = 0;
int motor2DirectionReverse = 1;

int motor1CounterSign = -1;
int motor2CounterSign = +1;


int motor1Speed = 0, motor2Speed = 0;
const int minAbsSpeed = 50;

int motor1QeiAPin = PB13; // purple
int motor1QeiBPin = PB12; // gray
int motor2QeiAPin = PB8; // purple
int motor2QeiBPin = PB9; // gray

const int motor1_IN1_Pin = PA11; // orange
const int motor1_IN2_Pin = PB15; // yellow
const int motor1_PWM_Pin = PA8; // green
const int motor1_StandBy_Pin = PB14; // blue

const int motor2_IN1_Pin = PB3; // orange
const int motor2_IN2_Pin = PB4; // yellow
const int motor2_PWM_Pin = PB6;  // green
const int motor2_StandBy_Pin = PB7; // blue

volatile int motor1QeiCounts = 0, motor2QeiCounts = 0;
volatile int motor1Counter = 0, motor2Counter = 0;

int motor1Direction = motor1DirectionForward;
int motor2Direction = motor2DirectionForward;

// command setup

int speedSetPoint = 0;
float distanceToCover = 0 ;
float coveredDistance = 0 ;
float distanceToDestination = 0 ;
int forwardAcceleration = 20 ; // acceleration 
int forwardDeceleration = 10 ;
int reverseDeceleration = 10 ;
int reverseAcceleration = 10 ;
int approachingSpeed = 50 ;

int robotRotating = 0 ; // -1 counterclockwise, 0 stopped, 1 counterclockwise
int robotDirection = 1 ; // -1 reverse, 0 stopped, 1 forward
float robotSpeed = 0 ; // 0 - 255 
float rof = 0; // roll offset

// Communications
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void dprint(String str)
{
	Serial.print(str);
}

void dprintln(String str)
{
	Serial.println(str);
}


void Motor1quickQEI()
{
	//a fast(ish) QEI function
	int state = 0;
	state = digitalRead(motor1QeiAPin) << 1;
	state = state|digitalRead(motor1QeiBPin);
	switch (state)
	{
	  case 0:
	    motor1QeiCounts--;
	    break;
	  case 1:
	    motor1QeiCounts++;
	    break;
	  case 2:
	    motor1QeiCounts++;
	    break;
	  case 3:
	    motor1QeiCounts--;
	    break;
	}
	motor1Counter = motor1QeiCounts * motor1CounterSign;
}

void Motor2quickQEI()
{
	//a fast(ish) QEI function
	int state = 0;
	state = digitalRead(motor2QeiAPin) << 1;
	state = state|digitalRead(motor2QeiBPin);
	switch (state)
	{
	  case 0:
	    motor2QeiCounts--;
	    break;
	  case 1:
	    motor2QeiCounts++;
	    break;
	  case 2:
	    motor2QeiCounts++;
	    break;
	  case 3:
	    motor2QeiCounts--;
	    break;
	}
	motor2Counter = motor2QeiCounts * motor2CounterSign;
}


void setup()
{
//	iwdg_init(IWDG_PRE_256, 10);
	// join I2C bus (I2Cdev library doesn't do this automatically)
	HWire.begin();

	pinMode(INTERRUPT_PIN, INPUT);

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(115200);
	cmdInit(&Serial);
	cmdAdd("set", set_param);
	cmdAdd("pid", get_PID);
	cmdAdd("ypr", get_YPR);
	cmdAdd("q", get_Q);
	cmdAdd("status", get_Status);
	cmdAdd("stop", emergency);

	// initialize device
	dprintln(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	dprintln(F("Testing device connections..."));
	dprintln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	delay(3000);

	// load and configure the DMP
	dprintln(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		dprintln(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		dprintln(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dprintln(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		dprint(F("DMP Initialization failed (code "));
		dprint(String(devStatus));
		dprintln(F(")"));
	}

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);

	//setup PID
	pid.SetMode(AUTOMATIC);
	pid.SetSampleTime(10);
	pid.SetOutputLimits(-255, 255);

	// setup motors
	pinMode(motor1QeiAPin, INPUT_PULLUP);
	pinMode(motor2QeiAPin, INPUT_PULLUP);
	pinMode(motor1QeiBPin, INPUT_PULLUP);
	pinMode(motor2QeiBPin, INPUT_PULLUP);
	motor1Wheel.setupPins(motor1_IN1_Pin, motor1_IN2_Pin, motor1_PWM_Pin, motor1_StandBy_Pin);
	motor2Wheel.setupPins(motor2_IN1_Pin, motor2_IN2_Pin, motor2_PWM_Pin, motor2_StandBy_Pin);

	attachInterrupt(motor1QeiAPin, Motor1quickQEI, CHANGE);
	attachInterrupt(motor2QeiAPin, Motor2quickQEI, CHANGE);

}

bool stop_mode = true;

void move(int speed)
{
	if(stop_mode) {
		motor1Wheel.stopMotor();
		motor2Wheel.stopMotor();
	} else {
		int direction = (speed < 0) ? -1 : 1;
		speed = abs(speed);
		speed = max(speed, minAbsSpeed);
		speed = min(speed, 255);
		motor1Wheel.setDirectionMode(direction);
		motor2Wheel.setDirectionMode(-direction);
		motor1Wheel.setMotorPower(direction * speed);
		motor2Wheel.setMotorPower(-direction * speed);
	}
}

void loop()
{
//	iwdg_feed();
	// if programming failed, don't try to do anything
	if (!dmpReady) {
		dprintln("DMP is not ready");
		delay(2000);
		return;
	}

	// wait for MPU interrupt or extra packet(s) available
	if(!mpuInterrupt && fifoCount < packetSize) {
		//no mpu data - performing PID calculations and output to motors
		pid.Compute();
		move(output);
	} else {

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			dprintln(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//			dprint("Pitch:");
//			dprintln(abs(ypr[1]));
			if(abs(ypr[1]) > 0.3) {
				stop_mode = true;
			} else {
				stop_mode = false;
				input = ypr[1] * 180/M_PI + 180 - rof;
			}

			#ifdef OUTPUT_READABLE_QUATERNION
				// display quaternion values in easy matrix form: w x y z
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				dprint("quat\t");
				dprint(q.w);
				dprint("\t");
				dprint(q.x);
				dprint("\t");
				dprint(q.y);
				dprint("\t");
				dprintln(q.z);
			#endif

			#ifdef OUTPUT_READABLE_EULER
				// display Euler angles in degrees
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetEuler(euler, &q);
				dprint("euler\t");
				dprint(euler[0] * 180/M_PI);
				dprint("\t");
				dprint(euler[1] * 180/M_PI);
				dprint("\t");
				dprintln(euler[2] * 180/M_PI);
			#endif

			#ifdef OUTPUT_READABLE_YAWPITCHROLL
				// display Euler angles in degrees
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				dprint("ypr\t");
				dprint(ypr[0] * 180/M_PI);
				dprint("\t");
				dprint(ypr[1] * 180/M_PI);
				dprint("\t");
				dprintln(ypr[2] * 180/M_PI);
			#endif

			#ifdef OUTPUT_READABLE_REALACCEL
				// display real acceleration, adjusted to remove gravity
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetAccel(&aa, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
				dprint("areal\t");
				dprint(aaReal.x);
				dprint("\t");
				dprint(aaReal.y);
				dprint("\t");
				dprintln(aaReal.z);
			#endif

			#ifdef OUTPUT_READABLE_WORLDACCEL
				// display initial world-frame acceleration, adjusted to remove gravity
				// and rotated based on known orientation from quaternion
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetAccel(&aa, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
				mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
				dprint("aworld\t");
				dprint(aaWorld.x);
				dprint("\t");
				dprint(aaWorld.y);
				dprint("\t");
				dprintln(aaWorld.z);
			#endif
		
			#ifdef OUTPUT_TEAPOT
				// display quaternion values in InvenSense Teapot demo format:
				teapotPacket[2] = fifoBuffer[0];
				teapotPacket[3] = fifoBuffer[1];
				teapotPacket[4] = fifoBuffer[4];
				teapotPacket[5] = fifoBuffer[5];
				teapotPacket[6] = fifoBuffer[8];
				teapotPacket[7] = fifoBuffer[9];
				teapotPacket[8] = fifoBuffer[12];
				teapotPacket[9] = fifoBuffer[13];
				Serial.write(teapotPacket, 14);
				teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
			#endif

			// blink LED to indicate activity
			blinkState = !blinkState;
			digitalWrite(LED_PIN, blinkState);
		}
	}
	cmdPoll();
//	if(stop_mode) {
//		dprintln("STOP MODE");
//	}
}

void display_error(String msg)
{
	Stream *s = cmdGetStream();
	s->println("{\"error\": \"" + msg + "\"}");
}

void display_ok(String msg="ok")
{
	Stream *s = cmdGetStream();
	s->println("{\"result\": \"" + msg + "\"}");
}

void set_param(int argc, char **argv)
{
	if(argc >= 3) {
		String param = String(argv[1]);
		float value = String(argv[2]).toFloat();
		if(param == "Kp") { Kp = value; display_ok(); }
		else if(param == "Kd") { Kd = value; display_ok(); }
		else if(param == "Ki") { Ki = value; display_ok(); }
		else if(param == "rof") { rof = value; display_ok(); }
		else display_error("bad parameter name:" + param);
	} else {
		display_error("format");
	}
}



//	else if (instruction == "vel") error = setSpeedSetPoint(value);
//	else if (instruction == "dir") error = setMotorDirection(value);
//	else if (instruction == "dis") error = setDistanceToCover(value);
//	else if (instruction == "rot") error = setRotationToValue(value);

void get_Status(int argc, char **argv)
{
	if(argc >= 2) {
		String type = String(argv[1]);
		char _buffer[300];
		if(type == "m1") {
			sprintf(_buffer, "{\"speed\":%d,\"cnt\":%d,\"dir\":%d}", motor1Speed, motor1Counter, motor1Direction == motor1DirectionForward);
			Serial.println(_buffer);
		} else if(type == "m2") {
			sprintf(_buffer, "{\"speed\":%d,\"cnt\":%d,\"dir\":%d}", motor2Speed, motor2Counter, motor2Direction == motor2DirectionForward);
			Serial.println(_buffer);
		} else if(type == "dist") {
			char float_str3[16] = "";
			dtostrf(distanceToDestination, 4, 2, float_str3);
			sprintf(_buffer, "{\"distleft\":%.2f}", distanceToDestination);
			Serial.println(_buffer);
		} else display_error("bad status type:" + type);
	} else {
		display_error("format");
	}
}

void get_PID(int argc, char **argv)
{
	Serial.print("{\"Kp\":");
	Serial.print(Kp);
	Serial.print(",\"Kd\":");
	Serial.print(Kd);
	Serial.print(",\"Ki\":");
	Serial.print(Ki);
	Serial.print(",\"rof\":");
	Serial.print(rof);
	Serial.println("}");
}

void get_YPR(int argc, char **argv)
{
	bool deg = true;
	if(argc > 1) {
		if(String(argv[1]) == "rad") {
			deg = false;
		}
	}
	Serial.print("{\"Y\":");
	Serial.print(ypr[0] * ((deg) ? 180/M_PI : 1));
	Serial.print(",\"P\":");
	Serial.print(ypr[1] * ((deg) ? 180/M_PI : 1));
	Serial.print(",\"R\":");
	Serial.print(ypr[2] * ((deg) ? 180/M_PI : 1));
	Serial.println("}");
}

void get_Q(int argc, char **argv)
{
	Serial.print("{\"QW\":");
	Serial.print(q.w);
	Serial.print(",\"QX\":");
	Serial.print(q.x);
	Serial.print(",\"QY\":");
	Serial.print(q.y);
	Serial.print(",\"QZ\":");
	Serial.print(q.z);
	Serial.println("}");
}

void emergency(int argc, char **argv)
{
	robotDirection = 0;
	robotRotating = 0 ;
	motor1Speed = 0 ;
	motor2Speed = 0 ;
	stop_mode = true;
	motor1Wheel.stopMotor();
	motor2Wheel.stopMotor();
	distanceToCover = 0 ;
}
