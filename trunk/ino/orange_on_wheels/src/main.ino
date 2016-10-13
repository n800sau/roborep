#include <voltage.h>
#include "SerialProtocol.h"
#include "commands.h"
#include "crc.h"
#include <EventFuse.h>

#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"
#include "angle.h"

#if defined(__AVR_ATmega2560__)
#else
#endif


// motor pins
const int LEFT_MOTOR_1 = 6;
const int LEFT_MOTOR_2 = 5;

const int RIGHT_MOTOR_1 = 10;
const int RIGHT_MOTOR_2 = 9;

const int echoPin = 12; // Echo Pin
const int trigPin = 11; // Trigger Pin

// encoder pins
const int Eleft = 2;
const int Eright = 3;

float stop_acc_x = 0;
float stop_acc_y = 0;
float stop_acc_z = 0;

// sonar
int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
const int MAX_STOP_DIST = 30;
long duration = -1, distance = -1; // Duration used to calculate distance

const float COUNT_PER_REV = 20.0;
const float WHEEL_DIAMETER = 0.065;
const float BASELINE = 0.14;
const float ENC_STEP = WHEEL_DIAMETER * PI / COUNT_PER_REV;

float count2dist(int count)
{
	return count * ENC_STEP;
}

const int MIN_PWM = 10;
const int MAX_PWM = 255;


// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder. 
//
// The threshold value I'm using limits it to 100 half pulses a second
//
// My encoder has 12 pulses per 360deg rotation and the specs say
// it is rated at a maximum of 100rpm.
//
// This threshold will permit my encoder to reach 250rpm so if it was connected
// to a motor instead of a manually operated knob I
// might possibly need to adjust it to 25000. However, this threshold
// value is working perfectly for my situation
//
volatile unsigned long threshold = 10000;

// Working variables for the interrupt routines
//
volatile unsigned long intLtime = 0;
volatile unsigned long intRtime = 0;
volatile int lCounter = 0;
volatile int rCounter = 0; 
int lPower = 0;
int rPower = 0;
// left + powerOffset
// right - powerOffset
int powerOffset = 0;
volatile bool lFwd = true;
volatile bool rFwd = true;
volatile bool full_stopped = true;
bool moving_straight = false;
int fwd_heading;

unsigned long last_ping = millis();


#define DEFAULT_PWR 70

void resetCounters()
{
	lCounter = rCounter = 0;
}

class IccBase: public SerialProtocol {
	public:
		void sendState();
		void ok();
		void error();

};

void IccBase::sendState()
{
	float vals[3] = {
		adxl345_state.event.acceleration.x - stop_acc_x,
		adxl345_state.event.acceleration.y - stop_acc_y,
		adxl345_state.event.acceleration.z - stop_acc_z
	};
	sendFloats(R_ACC_3F, vals, 3);

	vals[0] = readVccMv() / 1000.;
	sendFloats(R_VOLTS_1F, vals, 1);

	vals[0] = lCounter;
	vals[1] = rCounter;
	sendFloats(R_MCOUNTS_2F, vals, 2);

	vals[0] = lPower;
	vals[1] = rPower;
	sendFloats(R_MPOWER_2F, vals, 2);

	vals[0] = powerOffset;
	sendFloats(R_MPOWEROFFSET_1F, vals, 1);

	vals[0] = count2dist(lCounter);
	vals[1] = count2dist(rCounter);
	sendFloats(R_MDIST_2F, vals, 2);

	vals[0] = (distance > 0) ? distance / 100. : distance;
	sendFloats(R_DIST_1F, vals, 1);

	vals[0] = headingDegrees;
	sendFloats(R_HEADING_1F, vals, 1);

	vals[0] = compass_x;
	vals[1] = compass_y;
	vals[2] = compass_z;
	sendFloats(R_COMPASS_3F, vals, 3);

	vals[0] = acc_x_avg() - stop_acc_x;
	vals[1] = acc_y_avg() - stop_acc_y;
	vals[2] = acc_z_avg() - stop_acc_z;
	sendFloats(R_ACCAVG_3F, vals, 3);

	vals[0] = acc_x_max() - stop_acc_x;
	vals[1] = acc_y_max() - stop_acc_y;
	vals[2] = acc_z_max() - stop_acc_z;
	sendFloats(R_ACCMAX_3F, vals, 3);

//	vals[0] = adxl345_state.single_tap;
//	sendFloats(R_HIT_1F, vals, 1);

	vals[0] = gyro.g.x;
	vals[1] = gyro.g.y;
	vals[2] = gyro.g.z;
	sendFloats(R_GYRO_3F, vals, 3);

//	bmp.getTemperature(vals);
//	sendFloats(R_TEMPERATURE_1F, vals, 1);

//	vals[0] = bmp085_event.pressure;
//	sendFloats(R_PRESSURE_1F, vals, 1);

	sendFloats(R_END, NULL, 0);
}

void IccBase::ok()
{
	sendSimple(R_OK_0);
}

void IccBase::error()
{
	sendSimple(R_ERROR_0);
}

IccBase base;

void lIntCB()
{
	unsigned long t = micros();
	if( t - intLtime > threshold )
	{
		intLtime = t;
		lCounter += (lFwd) ? 1 : -1;
	}
}

void rIntCB()
{
	unsigned long t = micros();
	if( t - intRtime > threshold )
	{
		intRtime = t;
		rCounter += (rFwd) ? 1 : -1;
	}
}

int power2pwm(int power)
{
	int rs = MIN_PWM + (MAX_PWM-MIN_PWM) / 100. * power;
	if(rs < MIN_PWM) {
		rs = MIN_PWM;
	} else if(rs > MAX_PWM) {
		rs = MAX_PWM;
	}
	return rs;
}

void stop(bool full=false)
{
	if(full) {
		full_stopped = true;
	}
	moving_straight = false;
	powerOffset = 0;
	setLeftMotor(0, false);
	setRightMotor(0, false);
}

void straight(int pwr, bool fwd)
{
	setLeftMotor(pwr, fwd);
	setRightMotor(pwr, fwd);
	moving_straight = true;
	powerOffset = 0;
	fwd_heading = headingDegrees;
}

void setLeftMotor(int power, bool fwd)
{
	if(full_stopped) {
		power = 0;
	}
	lPower = power;
	if(moving_straight && !full_stopped) {
		power += powerOffset;
	}
	lFwd = fwd;
	if(power == 0) {
		digitalWrite(LEFT_MOTOR_1, LOW);
		digitalWrite(LEFT_MOTOR_2, LOW);
		Serial.println("Left stopped");
	} else {
		int pwm = power2pwm(power);
		if(fwd) {
			analogWrite(LEFT_MOTOR_1, pwm);
			digitalWrite(LEFT_MOTOR_2, LOW);
		} else {
			analogWrite(LEFT_MOTOR_2, pwm);
			digitalWrite(LEFT_MOTOR_1, LOW);
		}
		Serial.print("Left:");
		Serial.print(pwm);
		Serial.println((fwd) ? ", fwd": ", back");
	}
}

void setRightMotor(int power, bool fwd)
{
	if(full_stopped) {
		power = 0;
	}
	rPower = power;
	if(moving_straight && !full_stopped) {
		power -= powerOffset;
	}
	rFwd = fwd;
	if(power == 0) {
		digitalWrite(RIGHT_MOTOR_1, LOW);
		digitalWrite(RIGHT_MOTOR_2, LOW);
		Serial.println("Right stopped");
	} else {
		int pwm = power2pwm(power);
		if(fwd) {
			analogWrite(RIGHT_MOTOR_1, pwm);
			digitalWrite(RIGHT_MOTOR_2, LOW);
		} else {
			analogWrite(RIGHT_MOTOR_2, pwm);
			digitalWrite(RIGHT_MOTOR_1, LOW);
		}
		Serial.print("Right:");
		Serial.print(pwm);
		Serial.println((fwd) ? ", fwd": ", back");
	}
}

void sonar()
{
	noInterrupts();
	// The following trigPin/echoPin cycle is used to determine the
	// distance of the nearest object by bouncing soundwaves off of it.
	digitalWrite(trigPin, LOW); 
	delayMicroseconds(2); 

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10); 

	digitalWrite(trigPin, LOW);
	interrupts();
	duration = pulseIn(echoPin, HIGH);

	//Calculate the distance (in cm) based on the speed of sound.
	distance = duration/58.2;

	if(distance < minimumRange) {
		distance = minimumRange - 1;
	} else if(distance > maximumRange) {
		distance = maximumRange + 1;
	}
}

void serialEvent() {
	base.serialEvent();
}

void execute()
{
	switch(*base.command) {
		case C_MSTOP:
			stop(true);
			base.ok();
			break;
		case C_MLEFT:
			// bytes: power, direction, timeout
			if(*base.datasize >= 3) {
				full_stopped = false;
				setLeftMotor(base.dataBuf[0], base.dataBuf[1]);
				stop_after(base.dataBuf[2]);
				base.ok();
			} else {
				base.error();
			}
			break;
		case C_MRIGHT:
			// bytes: power, direction, timeout
			if(*base.datasize >= 3) {
				full_stopped = false;
				setRightMotor(base.dataBuf[0], base.dataBuf[1]);
				stop_after(base.dataBuf[2]);
				base.ok();
			} else {
				base.error();
			}
			break;
		case C_MBOTH:
			// bytes: lpower, lfwd, rpower, rfwd, timeout
			if(*base.datasize >= 5) {
				full_stopped = false;
				setLeftMotor(base.dataBuf[0], base.dataBuf[1]);
				setRightMotor(base.dataBuf[2], base.dataBuf[3]);
				stop_after(base.dataBuf[4]);
				base.ok();
			} else {
				base.error();
			}
			break;
		case C_WALK_AROUND:
			// bytes: power, timeout
			if(*base.datasize >= 2) {
				walk_around(base.dataBuf[0], base.dataBuf[1]);
			} else {
				base.error();
			}
			break;
		case C_MOVE2RELEASE:
			// bytes: power, fwd, timeout
			if(*base.datasize >= 3) {
				move2release(base.dataBuf[0], base.dataBuf[1], base.dataBuf[2]);
			} else {
				base.error();
			}
			break;
		case C_RESET_COUNTERS:
			resetCounters();
			base.ok();
			break;
		case C_STATE:
			base.sendState();
			break;
		case C_PING:
			base.ok();
			break;
	}
}


void setup()
{
	Serial.begin(115200);

	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);

	pinMode(LEFT_MOTOR_1, OUTPUT);
	pinMode(LEFT_MOTOR_2, OUTPUT);
	pinMode(RIGHT_MOTOR_1, OUTPUT);
	pinMode(RIGHT_MOTOR_2, OUTPUT);

	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);

	digitalWrite(Eleft, INPUT_PULLUP);
	digitalWrite(Eright, INPUT_PULLUP);

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	stop(true);
	delay(1000);

	process_compass();
	process_accel();
	process_gyro();
	process_bmp085();

	stop_acc_x = adxl345_state.event.acceleration.x;
	stop_acc_y = adxl345_state.event.acceleration.y;
	stop_acc_z = adxl345_state.event.acceleration.z;

	attachInterrupt(digitalPinToInterrupt(Eleft), rIntCB, RISING);
	attachInterrupt(digitalPinToInterrupt(Eright), lIntCB, RISING);

	EventFuse::newFuse(20, INF_REPEAT, evCommunicate);
	EventFuse::newFuse(50, INF_REPEAT, evSonar);
	EventFuse::newFuse(10, INF_REPEAT, evIMU);
	EventFuse::newFuse(2000, INF_REPEAT, evBMP85);
	EventFuse::newFuse(100, INF_REPEAT, evFixDir);

	Serial.println("Setup finished");

//	walk_around(DEFAULT_PWR, 10);
//	move2release(DEFAULT_PWR, false, 30);
}

void loop()
{
	EventFuse::burn();
}

