#include <Arduino.h>
#include <ros.h>

#define MAX_STOP_DIST 0.3

const int MIN_RANGE = 0.0;
const int MAX_RANGE = 2.0;

const int headEchoPin = 12; // Echo Pin
const int headTrigPin = 11; // Trigger Pin

const int SONAR_INCR = 5;
const int SONAR_CENTER_OFFSET = 10;

const int backEchoPin = 22; // Echo Pin
const int backTrigPin = 23; // Trigger Pin

// up-down
const int headTiltServoPin = 45;
// left-right
const int headServoPin = 46;

// encoder pins
const int Eleft = 2;
const int Eright = 3;

// motor pins
const int LEFT_MOTOR_1 = 6;
const int LEFT_MOTOR_2 = 5;

const int RIGHT_MOTOR_1 = 9;
const int RIGHT_MOTOR_2 = 10;

const float COUNT_PER_REV = 20.0;
const float WHEEL_DIAMETER = 0.065;
// distance between wheels
const float WHEEL_TRACK = 0.14;
const float ENC_STEP = WHEEL_DIAMETER * PI / COUNT_PER_REV;
const float TICKS_PER_METER = 1 / ENC_STEP;
const float GEAR_REDUCTION = 1.48; // from doc for chassis

const float PID_RATE = 10; // hz

const int MIN_PWM = 160;
const int MAX_PWM = 255;

#define DEFAULT_PWR 50

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
const unsigned long threshold = 10000;

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
void head_servo_move_to(int pos);

void updatePID();

