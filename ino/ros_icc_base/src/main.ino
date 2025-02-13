#include <voltage.h>
#include "SerialProtocol.h"
#include "commands.h"
#include "crc.h"

// sonar
int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
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

// encoder interrupts
const int lInt = Eleft - 2;
const int rInt = Eright - 2;

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
volatile uint8_t intLsignal = 0;
volatile uint8_t intRsignal = 0;
volatile uint8_t intLhistory = 0;
volatile uint8_t intRhistory = 0;
volatile int lCounter = 0;
volatile int rCounter = 0; 
volatile int lPower = 0;
volatile int rPower = 0; 

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
	float vals[3];
	vals[0] = readVccMv() / 1000.;
	sendFloats(R_VOLTS_1F, vals, 1);
	vals[0] = lCounter;
	vals[1] = rCounter;
	sendFloats(R_MCOUNTS_2F, vals, 2);
	vals[0] = lPower;
	vals[1] = rPower;
	sendFloats(R_MPOWER_2F, vals, 2);
	vals[0] = count2dist(lCounter);
	vals[1] = count2dist(rCounter);
	sendFloats(R_MDIST_2F, vals, 2);
	vals[0] = (distance > 0) ? distance / 100. : distance;
	sendFloats(R_DIST_1F, vals, 1);
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
	if( micros() - intLtime > threshold )
	{
		intLhistory = intLsignal;
		intLsignal = bitRead(PIND,Eright);
		if (intLhistory != intLsignal) {
			intLtime = micros();
			lCounter++;
		}
	}
}

void rIntCB()
{
	if( micros() - intRtime > threshold )
	{
		intRhistory = intRsignal;
		intRsignal = bitRead(PIND,Eleft);
		if (intRhistory != intRsignal) {
			intRtime = micros();
			rCounter++;
		}
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

void setLeftMotor(int power, bool fwd)
{
	lPower = power;
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
	rPower = power;
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
			setLeftMotor(0, false);
			setRightMotor(0, false);
			base.ok();
			break;
		case C_MLEFT:
			// bytes: power, direction
			if(*base.datasize >= 2) {
				setLeftMotor(base.dataBuf[0], base.dataBuf[1]);
				base.ok();
			} else {
				base.error();
			}
			break;
		case C_MRIGHT:
			// bytes: power, direction
			if(*base.datasize >= 2) {
				setRightMotor(base.dataBuf[0], base.dataBuf[1]);
				base.ok();
			} else {
				base.error();
			}
			break;
		case C_MBOTH:
			// bytes: lpower, lfwd, rpower, rfwd
			if(*base.datasize >= 4) {
				setLeftMotor(base.dataBuf[0], base.dataBuf[1]);
				setRightMotor(base.dataBuf[2], base.dataBuf[3]);
				base.ok();
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

	digitalWrite(Eleft, HIGH);
	digitalWrite(Eright, HIGH);

//	attachInterrupt(lInt, rIntCB, CHANGE);
//	attachInterrupt(rInt, lIntCB, CHANGE);

	Serial.println("Setup finished.");
}

unsigned long last_ping = millis();

void loop()
{
	sonar();
	if(base.available()) {
//		Serial.println("available");
		last_ping = millis();
		execute();
		base.resetInput();
	} else {
		if(last_ping + 60000 < millis()) {
			Serial.println("Control timeout. Stopping");
			setLeftMotor(0, false);
			setRightMotor(0, false);
		}
	}
	delay(50);
}

