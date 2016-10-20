#include "orange_on_wheels_ros.h"

#include <ros.h>

#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <fchassis_msgs/FCommand.h>

#include "angle.h"
#include <EventFuse.h>

#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"

const int echoPin = 12; // Echo Pin
const int trigPin = 11; // Trigger Pin

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
volatile unsigned long threshold = 10000;
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
String current_command;

using fchassis_msgs::FCommand;
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mf_msg;

ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::Publisher pub_imu( "/imu", &imu_msg);
ros::Publisher pub_mf( "/mf", &mf_msg);

void command_callback(const FCommand::Request & req, FCommand::Response & res) {
	if(strcmp(req.mcommand, "mstop") == 0) {
		stop(true);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "mleft") == 0) {
		current_command = req.mcommand;
		full_stopped = false;
		setLeftMotor(req.lPwr, req.lFwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "mright") == 0) {
		current_command = req.mcommand;
		full_stopped = false;
		setRightMotor(req.rPwr, req.rFwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "mboth") == 0) {
		current_command = req.mcommand;
		full_stopped = false;
		setLeftMotor(req.lPwr, req.lFwd);
		setRightMotor(req.rPwr, req.rFwd);
		stop_after(req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "walk_around") == 0) {
		current_command = req.mcommand;
		walk_around(req.pwr, req.timeout);
		res.reply = "ok";
	} else if(strcmp(req.mcommand, "move2release") == 0) {
		current_command = req.mcommand;
		move2release(req.pwr, req.fwd, req.timeout);
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

	setup_compass();
	setup_accel();
	setup_gyro();

	attachInterrupt(digitalPinToInterrupt(Eleft), rIntCB, RISING);
	attachInterrupt(digitalPinToInterrupt(Eright), lIntCB, RISING);

	EventFuse::newFuse(100, INF_REPEAT, evFixDir);

	nh.initNode();
	nh.advertise(pub_range);
	nh.advertise(pub_imu);
	nh.advertise(pub_mf);
	nh.advertiseService(server);

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.header.frame_id = range_frameid;
	range_msg.field_of_view = 0.1; // fake
	range_msg.min_range = 0.0;
	range_msg.max_range = 2.0;

	imu_msg.header.frame_id = imu_frameid;

}


unsigned long msg_time = 0;

void loop()
{
	EventFuse::burn();
	//publish the adc value every 50 milliseconds
	//since it takes that long for the sensor to stablize
	if ( millis() >= msg_time ){
				range_msg.range = getRange_Ultrasound();
				range_msg.header.stamp = nh.now();
				pub_range.publish(&range_msg);
				process_compass();
				mf_msg.header.stamp = nh.now();
				mf_msg.magnetic_field.x = compass_x;
				mf_msg.magnetic_field.y = compass_y;
				mf_msg.magnetic_field.z = compass_z;
				pub_mf.publish(&mf_msg);
				process_accel();
				process_gyro();
				imu_msg.header.stamp = nh.now();
				imu_msg.angular_velocity.x = gyro.g.x;
				imu_msg.angular_velocity.y = gyro.g.y;
				imu_msg.angular_velocity.z = gyro.g.z;
				imu_msg.linear_acceleration.x = acc_x_avg();
				imu_msg.linear_acceleration.y = acc_y_avg();
				imu_msg.linear_acceleration.z = acc_z_avg();
				pub_imu.publish(&imu_msg);
		msg_time = millis() + 50;
	}
	nh.spinOnce();
}
