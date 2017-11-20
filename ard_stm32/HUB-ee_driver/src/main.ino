#include <PID_v1.h>
#include <Cmd.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HardWire.h"
//#include "Wire.h"
#include <HUBeeBMDWheel.h>
//#include <libmaple/iwdg.h>

// wheel dimension
const float WL_CountPerRev = 128;
const float WL_Diameter = 0.06;
const float WL_Width = 0.02;
const float WL_Step = WL_Diameter * PI / WL_CountPerRev;

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

volatile bool mpuInterrupt = false;	 // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
	mpuInterrupt = true;
}

//PID

//adjust these values to fit your own design
double Kp = 20000;
double Kd = 1000;
double Ki = 1000;

double m1_vel = 0;
double m1_curvel, m1_power;
PID m1_pid(&m1_curvel, &m1_power, &m1_vel, Kp, Ki, Kd, DIRECT);

double m2_vel = 0;
double m2_curvel, m2_power;
PID m2_pid(&m2_curvel, &m2_power, &m2_vel, Kp, Ki, Kd, DIRECT);

// HUB-ee wheels

HUBeeBMDWheel motor1Wheel;
HUBeeBMDWheel motor2Wheel;

int motor1Power = 0;
int motor2Power = 0;

const int motor1CounterSign = -1;
const int motor2CounterSign = +1;

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

// raw counter
volatile int motor1QeiCounts = 0, motor2QeiCounts = 0;
// counter with sign
volatile int motor1Counter = 0, motor2Counter = 0;

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

bool stop_mode = true, last_stop_mode = true;
int last_motor1Counter, last_motor2Counter;
unsigned long last_ts;

void setup()
{
//	iwdg_init(IWDG_PRE_256, 10);
	// join I2C bus (I2Cdev library doesn't do this automatically)
	HWire.begin();

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(115200);

	//setup left PID
	m1_pid.SetMode(AUTOMATIC);
	m1_pid.SetSampleTime(10);
	m1_pid.SetOutputLimits(-255, 255);

	//setup right PID
	m2_pid.SetMode(AUTOMATIC);
	m2_pid.SetSampleTime(10);
	m2_pid.SetOutputLimits(-255, 255);

	pinMode(INTERRUPT_PIN, INPUT);

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

	// setup motors
	pinMode(motor1QeiAPin, INPUT_PULLUP);
	pinMode(motor2QeiAPin, INPUT_PULLUP);
	pinMode(motor1QeiBPin, INPUT_PULLUP);
	pinMode(motor2QeiBPin, INPUT_PULLUP);
	motor1Wheel.setupPins(motor1_IN1_Pin, motor1_IN2_Pin, motor1_PWM_Pin, motor1_StandBy_Pin);
	motor2Wheel.setupPins(motor2_IN1_Pin, motor2_IN2_Pin, motor2_PWM_Pin, motor2_StandBy_Pin);

	attachInterrupt(motor1QeiAPin, Motor1quickQEI, CHANGE);
	attachInterrupt(motor2QeiAPin, Motor2quickQEI, CHANGE);

	cmdInit(&Serial);
	cmdAdd("set", set_param);
	cmdAdd("pid", get_PID);
	cmdAdd("ypr", get_YPR);
	cmdAdd("q", get_Q);
	cmdAdd("status", get_Status);
	cmdAdd("go", go);
	cmdAdd("stop", full_stop);

//0.05 max 0.02 min
//	m1_vel = m2_vel = -0.02;
//	stop_mode = false;
}

void move()
{
	if(last_stop_mode && !stop_mode) {
		// start moving
		last_motor1Counter = motor1Counter = 0;
		last_motor2Counter = motor2Counter = 0;
		last_ts = millis();
	}
	if(stop_mode) {
		motor1Wheel.stopMotor();
		motor2Wheel.stopMotor();
	} else {
		motor1Wheel.setMotorPower(-motor1Power);
		motor2Wheel.setMotorPower(motor2Power);
	}
	last_stop_mode = stop_mode;
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
		// calculate speeds
		unsigned long ts = millis();
		unsigned long ts_diff = ts - last_ts;
		if(ts_diff > 0) {
			// m/s
			m1_curvel = double(motor1Counter - last_motor1Counter) * WL_Step / ts_diff * 1000;
			m2_curvel = double(motor2Counter - last_motor2Counter) * WL_Step / ts_diff * 1000;

//Serial.print("m1_curvel:");
//Serial.print(m1_curvel, 8);
//Serial.print(",m2_curvel:");
//Serial.println(m2_curvel, 8);

//			if(ts_diff > 100) {

//				last_ts = ts;
//				last_motor1Counter = motor1Counter;
//				last_motor2Counter = motor2Counter;
//			}

			m1_pid.Compute();
			m2_pid.Compute();

			motor1Power = m1_power;
			motor2Power = m2_power;

			move();

		}
		cmdPoll();
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

			// blink LED to indicate activity
			blinkState = !blinkState;
			digitalWrite(LED_PIN, blinkState);
		}
	}
}

void display_error(String msg)
{
	Stream *s = cmdGetStream();
	s->println("{\"error\":\"" + msg + "\"}");
}

void display_ok(String msg="ok")
{
	Stream *s = cmdGetStream();
	s->println("{\"result\":\"" + msg + "\"}");
}

void display_param(String name, float value)
{
	Stream *s = cmdGetStream();
	s->println("{\"name\":\"" + name + "\",\"value\":" + value + "}");
}

void set_param(int argc, char **argv)
{
	if(argc >= 3) {
		String param = String(argv[1]);
		float value = String(argv[2]).toFloat();
		if(param == "Kp") { Kp = value; display_param("Kp", Kp); }
		else if(param == "Kd") { Kd = value; display_param("Kd", Kd); }
		else if(param == "Ki") { Ki = value; display_param("Ki", Ki); }
		else if(param == "vel") { m1_vel = m2_vel = value; display_param("vel", value); }
		else if(param == "m1vel") { m1_vel = value; display_param("m1vel", m1_vel); }
		else if(param == "m2vel") { m2_vel = value; display_param("m2vel", m2_vel); }
		else display_error("bad parameter name:" + param);
	} else {
		display_error("format");
	}
}

void get_Status(int argc, char **argv)
{
	String reply = String("{\"m1\":") +
		"{\"pwr\":" + String(motor1Power) + ",\"cnt\":" + String(motor1Counter) +
			",\"v\":" + String(m1_curvel, 8) + ",\"vp\":" + m1_vel + "},\"m2\":" +
		"{\"pwr\":" + String(motor2Power) + ",\"cnt\":" + String(motor2Counter) +
			",\"v\":" + String(m2_curvel, 8) + ",\"vp\":" + m2_vel +  "}}";
	Serial.println(reply);
}

void get_PID(int argc, char **argv)
{
	Serial.print("{\"Kp\":");
	Serial.print(Kp);
	Serial.print(",\"Kd\":");
	Serial.print(Kd);
	Serial.print(",\"Ki\":");
	Serial.print(Ki);
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
	Serial.print(ypr[0] * ((deg) ? 180/M_PI : 1), 3);
	Serial.print(",\"P\":");
	Serial.print(ypr[1] * ((deg) ? 180/M_PI : 1), 3);
	Serial.print(",\"R\":");
	Serial.print(ypr[2] * ((deg) ? 180/M_PI : 1), 3);
	Serial.println("}");
}

void get_Q(int argc, char **argv)
{
	Serial.print("{\"QW\":");
	Serial.print(q.w, 3);
	Serial.print(",\"QX\":");
	Serial.print(q.x, 3);
	Serial.print(",\"QY\":");
	Serial.print(q.y, 3);
	Serial.print(",\"QZ\":");
	Serial.print(q.z, 3);
	Serial.println("}");
}

void full_stop(int argc, char **argv)
{
	motor1Power = 0;
	motor2Power = 0;
	m1_vel = m2_vel = 0;
	stop_mode = true;
	motor1Wheel.stopMotor();
	motor2Wheel.stopMotor();
	display_ok();
}

void go(int argc, char **argv)
{
	stop_mode = false;
	display_ok();
}
