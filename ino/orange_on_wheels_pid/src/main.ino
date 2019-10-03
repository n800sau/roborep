#include "orange_on_wheels_pid.h"

#include <EventFuse.h>
#include <Servo.h>

volatile int lCounter = 0;
volatile int rCounter = 0; 
int lDirection = 0;
int rDirection = 0; 

// linear velocity m/s
float linVel = 0;
float lvStep = 0.01;
// angular velocity rad/s
float angVel = 0;
float avStep = 0.01;

volatile float lVel = 0;
volatile float rVel = 0; 

int sonarAngle = 90;
int sonarIncr = SONAR_INCR;
unsigned long last_head_servo_move_ts = 0;
Servo head_servo;

bool full_stopped = true;

void setup()
{
	Serial.begin(115200);
	// for ir commands
	Serial3.begin(9600);
	pinMode(headTrigPin, OUTPUT);
	pinMode(headEchoPin, INPUT);
	pinMode(backTrigPin, OUTPUT);
	pinMode(backEchoPin, INPUT);

	pinMode(LEFT_MOTOR_1, OUTPUT);
	pinMode(LEFT_MOTOR_2, OUTPUT);
	pinMode(RIGHT_MOTOR_1, OUTPUT);
	pinMode(RIGHT_MOTOR_2, OUTPUT);

	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);

	digitalWrite(Eleft, INPUT_PULLUP);
	digitalWrite(Eright, INPUT_PULLUP);

	head_servo_move_to(90 + SONAR_CENTER_OFFSET);

	attachInterrupt(digitalPinToInterrupt(Eleft), rIntCB, RISING);
	attachInterrupt(digitalPinToInterrupt(Eright), lIntCB, RISING);

	EventFuse::newFuse(2000, INF_REPEAT, evHeadServoDetach);
	EventFuse::newFuse(200, INF_REPEAT, evIRcmd);

//	full_stopped = false;
//	setLeftMotor(50);
//	setRightMotor(50);
//	stop_after(400);

/*			analogWrite(LEFT_MOTOR_1, 200);
			digitalWrite(LEFT_MOTOR_2, LOW);
			analogWrite(RIGHT_MOTOR_1, 200);
			digitalWrite(RIGHT_MOTOR_2, LOW);
//			digitalWrite(RIGHT_MOTOR_1, LOW);
//			digitalWrite(RIGHT_MOTOR_2, LOW);
	delay(1000);
			digitalWrite(LEFT_MOTOR_1, LOW);
			digitalWrite(LEFT_MOTOR_2, LOW);
			analogWrite(RIGHT_MOTOR_1, 200);
			digitalWrite(RIGHT_MOTOR_2, LOW);
	delay(1000);
			digitalWrite(LEFT_MOTOR_1, LOW);
			analogWrite(LEFT_MOTOR_2, 200);
			digitalWrite(RIGHT_MOTOR_1, LOW);
			digitalWrite(RIGHT_MOTOR_2, LOW);
	delay(1000);
			digitalWrite(LEFT_MOTOR_1, LOW);
			digitalWrite(LEFT_MOTOR_2, LOW);
			digitalWrite(RIGHT_MOTOR_1, LOW);
			analogWrite(RIGHT_MOTOR_2, 200);
	delay(1000);
			digitalWrite(LEFT_MOTOR_1, LOW);
			digitalWrite(LEFT_MOTOR_2, LOW);
			digitalWrite(RIGHT_MOTOR_1, LOW);
			digitalWrite(RIGHT_MOTOR_2, LOW);
*/
}

void loop()
{
	EventFuse::burn();
}

