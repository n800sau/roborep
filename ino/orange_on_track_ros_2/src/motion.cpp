#include "motion.h"
#include "encoders.h"
#include "const.h"
#include <PID_v1.h>
#include "ros_handle.h"

namespace motion {

	// motor pins
	const int LEFT_MOTOR_FWD = 6;
	const int LEFT_MOTOR_POWER = 5;

	const int RIGHT_MOTOR_FWD = 9;
	const int RIGHT_MOTOR_POWER = 10;

	float left, right;
	double lIn, rIn; // PID Input Signal
	double lOut, rOut; // PID Output command for each wheel
	double lSet=0, rSet=0; // PID Required speed for each wheel
	const double lkp=0.5, lki=10, lkd=0.0;          // Left/right wheel PID constants.
	const double rkp=0.5, rki=10, rkd=0.0;
	const int PID_PERIOD = 200;                                   // PID period in milliseconds
	const double PID_RATE = 1000./PID_PERIOD;                             // number of periods/sec, PID rate
	PID lPID(&lIn, &lOut, &lSet, lkp, lki, lkd, DIRECT);
	PID rPID(&rIn, &rOut, &rSet, rkp, rki, rkd, DIRECT);
	unsigned long current_time;
	unsigned long last_time;
	int last_lCounter = 0;
	int last_rCounter = 0;
	bool full_stop = true;

	// move motor at pwm power and change directions flags only when motor cross stop
	void setMotors(int lpwm, int rpwm)
	{
		int llevel, rlevel;
		if(lpwm<1) {
			encoders::lFwd = 0;
			llevel = LOW;
		} else {
			encoders::lFwd = 1;
			llevel = HIGH;
		}
		if(rpwm<1) {
			encoders::rFwd = 0;
			rlevel = LOW;
		} else {
			encoders::rFwd = 1;
			rlevel = HIGH;
		}
		if(full_stop) {
			lpwm = rpwm = 0;
//			nh.loginfo("Full stop");
		}
		if(lpwm != 0 || rpwm != 0) {
			nh.loginfo(("motion lpwm:" + String(((llevel == LOW) ? -1 : 1) * lpwm) + ", rpwm:" + String(((rlevel == LOW) ? -1 : 1) * rpwm)).c_str());
		}
		analogWrite(LEFT_MOTOR_POWER, abs(lpwm));
		digitalWrite(LEFT_MOTOR_FWD, llevel);
		analogWrite(RIGHT_MOTOR_POWER, abs(rpwm));
		digitalWrite(RIGHT_MOTOR_FWD, rlevel);
	}

	void stop()
	{
		nh.loginfo("Stop");
		full_stop = true;
		lSet = rSet = lIn = rIn = lOut = rOut = left = right = 0;
		lPID.SetMode(MANUAL);
		rPID.SetMode(MANUAL);
		setMotors(0, 0);
	}

	void setup()
	{
		pinMode(LEFT_MOTOR_POWER, OUTPUT);
		pinMode(LEFT_MOTOR_FWD, OUTPUT);
		pinMode(RIGHT_MOTOR_POWER, OUTPUT);
		pinMode(RIGHT_MOTOR_FWD, OUTPUT);
		lPID.SetSampleTime(PID_PERIOD);
		rPID.SetSampleTime(PID_PERIOD);
		lPID.SetOutputLimits(-255, 255);
		rPID.SetOutputLimits(-255, 255);
		lPID.SetMode(AUTOMATIC);
		rPID.SetMode(AUTOMATIC);
		last_time = millis();
	}

	void tick()
	{
		current_time = millis();
		double dt = current_time - last_time;
		// speed in encoder ticks in sec
		double lvt = (encoders::lCounter - last_lCounter) / dt * 1000;
		double rvt = (encoders::rCounter - last_rCounter) / dt * 1000;
		last_time = current_time;
		last_lCounter = encoders::lCounter;
		last_rCounter = encoders::rCounter;
		if(lvt != 0 || rvt != 0) {
			nh.loginfo(("motion lcnt:" + String(encoders::lCounter) + ", rcnt:" + String(encoders::rCounter) + ", dt:" + String(dt)).c_str());
			nh.loginfo(("motion lvt:" + String(lvt, 2) + ", rvt:" + String(rvt, 2)).c_str());
		}
		// if movement occurs
		// use timing to calculate velocity only if ticks are greater than one
		// to avoid problems with sign and inertia for a simple encoder (no quadrature)	
//		if(abs(lvt)>=1) {
			lIn=lvt;
			lPID.Compute();
//			nh.loginfo("left compute");
//		}
//		if(abs(rvt)>=1) {
			rIn=rvt;
			rPID.Compute();
//			nh.loginfo("right compute");
//		}
		setMotors(lOut, rOut);
	}

	void set_motion(float x, float th)
	{
		if(x == 0 && th == 0) {
			stop();
		} else {
			full_stop = false;
			if(x == 0) {
				// Turn in place
				right = th * WHEEL_TRACK / 2.0;
				left = -right;
			} else if(th == 0) {
				// Pure forward/backward motion
				left = right = x;
			} else {
				// Rotation about a point in space
				left = x - th * WHEEL_TRACK / 2.0;
				right = x + th * WHEEL_TRACK / 2.0;
			}
			nh.loginfo(("Left:" + String(left, 2)).c_str());
			nh.loginfo(("Right:" + String(right, 2)).c_str());
			// target - encoder counts per sec
			lSet = round(left * TICKS_PER_METER);
			rSet = round(right * TICKS_PER_METER);
			nh.loginfo(("lSet:" + String(lSet)).c_str());
			nh.loginfo(("rSet:" + String(rSet)).c_str());
			lPID.SetMode(AUTOMATIC);
			rPID.SetMode(AUTOMATIC);
		}
	}

}