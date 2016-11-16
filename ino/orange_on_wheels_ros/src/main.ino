#include "orange_on_wheels_ros.h"

#include <ros.h>

#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <fchassis_srv/FCommand.h>
#include <fchassis_srv/mstate.h>

#include "angle.h"
#include <voltage.h>
#include <EventFuse.h>

#include <Servo.h>

#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"

const int echoPin = 12; // Echo Pin
const int trigPin = 11; // Trigger Pin

int sonarAngle = 90;
const int SONAR_INCR = 5;
int sonarIncr = SONAR_INCR;
const int SONAR_CENTER_OFFSET = 10;

const int bEchoPin = 22; // Echo Pin
const int bTrigPin = 23; // Trigger Pin

const int headServoPin = 46;

// encoder pins
const int Eleft = 2;
const int Eright = 3;

// motor pins
const int LEFT_MOTOR_1 = 10;
const int LEFT_MOTOR_2 = 9;

const int RIGHT_MOTOR_1 = 6;
const int RIGHT_MOTOR_2 = 5;

const float COUNT_PER_REV = 20.0;
const float WHEEL_DIAMETER = 0.065;
const float BASELINE = 0.14;
const float ENC_STEP = WHEEL_DIAMETER * PI / COUNT_PER_REV;

const int MIN_PWM = 10;
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
volatile unsigned long intLtime = 0;
volatile unsigned long intRtime = 0;
volatile int lCounter = 0;
volatile int rCounter = 0; 
volatile float lVel = 0;
volatile float rVel = 0; 
volatile int lPower = 0;
volatile int rPower = 0;
// left + powerOffset
// right - powerOffset
volatile int powerOffset = 0;
volatile bool lFwd = true;
volatile bool rFwd = true;
volatile bool full_stopped = true;
bool moving_straight = false;
int fwd_heading;
char current_command[20];

Servo head_servo;

using fchassis_srv::FCommand;
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mf_msg;
fchassis_srv::mstate state_msg;
std_msgs::Int16 lwheel_msg;
std_msgs::Int16 rwheel_msg;

ros::Publisher pub_range( "/fchassis/sonar", &range_msg);
ros::Publisher pub_imu( "/fchassis/imu", &imu_msg);
ros::Publisher pub_mf( "/fchassis/mf", &mf_msg);
ros::Publisher pub_lwheel("/fchassis/lwheel", &lwheel_msg);
ros::Publisher pub_rwheel("/fchassis/rwheel", &rwheel_msg);
ros::Publisher pub_state( "/fchassis/state", &state_msg);

void command_callback(const FCommand::Request & req, FCommand::Response & res) {
	res.timeout = req.timeout;
	if(strcmp(req.mcommand, "mstop") == 0) {
		strncpy(current_command, req.mcommand, sizeof(current_command));
		stop(true);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "mleft") == 0) {
		strncpy(current_command, req.mcommand, sizeof(current_command));
		full_stopped = false;
		setLeftMotor(req.lPwr, req.lFwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "mright") == 0) {
		strncpy(current_command, req.mcommand, sizeof(current_command));
		full_stopped = false;
		setRightMotor(req.rPwr, req.rFwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "mboth") == 0) {
		strncpy(current_command, req.mcommand, sizeof(current_command));
		full_stopped = false;
		setLeftMotor(req.lPwr, req.lFwd);
		setRightMotor(req.rPwr, req.rFwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "walk_around") == 0) {
		strncpy(current_command, req.mcommand, sizeof(current_command));
		full_stopped = false;
		straight(req.pwr, true);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "move2release") == 0) {
		strncpy(current_command, req.mcommand, sizeof(current_command));
		full_stopped = false;
		move2release(req.pwr, req.fwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "reset_counters") == 0) {
		resetCounters();
		res.reply = "ok";
	} else {
		res.error = true;
		res.reply = "command not found";
	}
}

ros::ServiceServer<FCommand::Request, FCommand::Response> server("exec_command",&command_callback);

const char range_frameid[] = "/ultrasound";
const char imu_frameid[] = "/imu";
const char base_frameid[] = "/base_link";

void lIntCB()
{
	unsigned long t = micros();
	if( t - intLtime > threshold )
	{
		int step = (lFwd) ? 1 : -1;
		lVel = step * ENC_STEP/t;
		intLtime = t;
		lCounter += step;
	}
}

void rIntCB()
{
	unsigned long t = micros();
	if( t - intRtime > threshold )
	{
		int step = (rFwd) ? 1 : -1;
		rVel = step * ENC_STEP/t;
		intRtime = t;
		rCounter += step;
	}
}

void setup()
{
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

	head_servo.attach(headServoPin);
	head_servo.write(90 + SONAR_CENTER_OFFSET);
//	head_servo.detach();

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	attachInterrupt(digitalPinToInterrupt(Eleft), rIntCB, RISING);
	attachInterrupt(digitalPinToInterrupt(Eright), lIntCB, RISING);

	EventFuse::newFuse(100, INF_REPEAT, evSonar);
	EventFuse::newFuse(100, INF_REPEAT, evFixDir);

//	EventFuse::newFuse(100, INF_REPEAT, evMoveSonar);

	nh.initNode();
	nh.advertise(pub_range);
	nh.advertise(pub_imu);
	nh.advertise(pub_mf);
	nh.advertise(pub_state);
	nh.advertise(pub_lwheel);
	nh.advertise(pub_rwheel);
	nh.advertiseService(server);

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.header.frame_id = range_frameid;
	range_msg.field_of_view = 0.1; // fake
	range_msg.min_range = 0.0;
	range_msg.max_range = 2.0;

	imu_msg.header.frame_id = imu_frameid;
	imu_msg.orientation_covariance[0] = -1;
	imu_msg.orientation_covariance[4] = -1;
	imu_msg.orientation_covariance[8] = -1;
	imu_msg.angular_velocity_covariance[0] = 0.1;
	imu_msg.angular_velocity_covariance[4] = 0.1;
	imu_msg.angular_velocity_covariance[8] = 0.1;
	imu_msg.linear_acceleration_covariance[0] = 0.1;
	imu_msg.linear_acceleration_covariance[4] = 0.1;
	imu_msg.linear_acceleration_covariance[8] = 0.1;

	mf_msg.header.frame_id = imu_frameid;

	state_msg.header.frame_id = base_frameid;
}


unsigned long msg_time = 0;

//publish values every 10 milliseconds
#define PUBLISH_PERIOD 10

void loop()
{
	EventFuse::burn();
	unsigned long m = millis();
	if ( m >= msg_time ){
		ros::Time now = nh.now();
		range_msg.range = getRange_Ultrasound();
		range_msg.header.stamp = now;
		pub_range.publish(&range_msg);
		process_compass();
		mf_msg.header.stamp = now;
		mf_msg.magnetic_field.x = compass_x;
		mf_msg.magnetic_field.y = compass_y;
		mf_msg.magnetic_field.z = compass_z;
		pub_mf.publish(&mf_msg);
		process_accel();
		process_gyro();
		imu_msg.header.stamp = now;
		imu_msg.angular_velocity.x = gyro.g.x * PI / 180;
		imu_msg.angular_velocity.y = gyro.g.y * PI / 180;
		imu_msg.angular_velocity.z = gyro.g.z * PI / 180;
		imu_msg.linear_acceleration.x = adxl345_state.event.acceleration.x;
		imu_msg.linear_acceleration.y = adxl345_state.event.acceleration.y;
		imu_msg.linear_acceleration.z = adxl345_state.event.acceleration.z;
		pub_imu.publish(&imu_msg);

		process_bmp085();
		state_msg.header.stamp = now;
		state_msg.v = readVccMv() / 1000.;
		state_msg.lcount = lCounter;
		state_msg.rcount = rCounter;
		state_msg.lpwr = lPower * ((lFwd) ? 1 : -1);
		state_msg.rpwr = rPower * ((rFwd) ? 1 : -1);
		state_msg.pwroffset = powerOffset;
		state_msg.ldist = count2dist(lCounter);
		state_msg.rdist = count2dist(rCounter);
		state_msg.heading = headingDegrees;
		state_msg.single_tap = adxl345_state.single_tap;
		float temp;
		bmp.getTemperature(&temp);
		state_msg.t = temp;
		state_msg.pressure = bmp085_event.pressure;
		state_msg.command = current_command;
		pub_state.publish(&state_msg);
		lwheel_msg.data = lCounter;
		rwheel_msg.data = rCounter;
		pub_lwheel.publish(&lwheel_msg);
		pub_rwheel.publish(&rwheel_msg);
		msg_time = millis() + PUBLISH_PERIOD;
	}
//	Servo::refresh();
	nh.spinOnce();
}
