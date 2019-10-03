#include <Arduino.h>
#include <ros.h>

#define MAX_STOP_DIST 0.3

const int MIN_RANGE = 0.05;
const int MAX_RANGE = 2.0;

const int headEchoPin = 12; // Echo Pin
const int headTrigPin = 11; // Trigger Pin

const int SONAR_INCR = 5;

const int SONAR_PAN_ANGLE_MIN = 35;
const int SONAR_PAN_ANGLE_MAX = 135;
const int SONAR_PAN_CENTER = SONAR_PAN_ANGLE_MIN + (SONAR_PAN_ANGLE_MAX - SONAR_PAN_ANGLE_MIN) / 2;

const int SONAR_TILT_ANGLE_MIN = 80;
const int SONAR_TILT_ANGLE_MAX = 145;
const int SONAR_TILT_CENTER = SONAR_TILT_ANGLE_MIN + (SONAR_TILT_ANGLE_MAX - SONAR_TILT_ANGLE_MIN) / 2;

const int backEchoPin = 22; // Echo Pin
const int backTrigPin = 23; // Trigger Pin

// up-down
const int headTiltServoPin = 45;
// left-right
const int headPanServoPin = 46;

// encoder pins
const int Eleft = 2;
const int Eright = 3;

// mpu6050 int pin
const int MPUint = 18;

// motor pins
const int LEFT_MOTOR_FWD = 6;
const int LEFT_MOTOR_POWER = 5;

const int RIGHT_MOTOR_FWD = 9;
const int RIGHT_MOTOR_POWER = 10;

const float COUNT_PER_REV = 70.0; // truck wheel 18 stripes (625 edges per rev)
const float WHEEL_DIAMETER = 0.05; // truck wheels
// distance between wheels
const float WHEEL_TRACK = 0.082; // truck width
const float ENC_STEP = WHEEL_DIAMETER * PI / COUNT_PER_REV;
const float TICKS_PER_METER = 1 / ENC_STEP;
const float GEAR_REDUCTION = 50 / 12. * 50 / 12.; // from doc for chassis ???

const float PID_RATE = 10; // hz

const int MIN_PWM = 15;
const int MAX_PWM = 255;

extern volatile bool cmd_vel_mode;
extern volatile bool full_stopped;
extern volatile int lCounter;
extern volatile int rCounter;

extern ros::NodeHandle  nh;

/* PID setpoint info For a Motor */
typedef struct {
	double TargetTicksPerFrame;    // target speed in ticks per frame
	long Encoder;                  // encoder count
	long PrevEnc;                  // last encoder count

	/*
	* Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
	* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
	*/
	int PrevInput;                // last input
	//int PrevErr;                   // last error

	/*
	* Using integrated term (ITerm) instead of integrated error (Ierror),
	* to allow tuning changes,
	* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
	*/
	//int Ierror;
	int ITerm;                    //integrated term

	long output;                    // last motor setting
}
SetPointInfo;

extern SetPointInfo leftPID, rightPID;
void resetPID();

void stop(bool full=false);
void setLeftMotor(int power, bool fwd);
void setRightMotor(int power, bool fwd);
void stop_after(int timeout);
void move2release(int pwr, bool fwd);
void straight(int pwr, bool fwd);
void resetCounters();
float getRange_HeadUltrasound(int attempts=2);
float getRange_BackUltrasound(int attempts=2);
void head_pan_servo_move_to(int pos);
void head_tilt_servo_move_to(int pos);

void updatePID();

extern ros::NodeHandle  nh;

template <class type> void getParam(const char *name, type *ptr, int count)
{
	bool param_success = false;
	do {
		param_success = nh.getParam(name, ptr, count);
		nh.spinOnce();
	} while(!param_success);
}

