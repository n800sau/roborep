#include "orange_on_track_ros.h"

#include <limits.h>

#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <fchassis_srv/FCommand.h>
#include <fchassis_srv/FTwistScan.h>
#include <fchassis_srv/mstate.h>
#include <fchassis_srv/AngleRange.h>
#include <fchassis_srv/FScannerSwitch.h>
#include <fchassis_srv/FScannerSetDirection.h>
#include <stuck_detector/Stuck.h>

#include "angle.h"
#include <voltage.h>
#include <EventFuse.h>
#include <Servo.h>

#include "hmc5883l_proc.h"
#include "adxl345_proc.h"
#include "l3g4200d_proc.h"
#include "bmp085_proc.h"


int sonarAngle = 90;
int sonarIncr = SONAR_INCR;

volatile int lCounter = 0;
volatile int rCounter = 0; 
volatile int lPower = 0;
volatile int rPower = 0;
// left + pid_powerOffset
// right - pid_powerOffset
volatile int pid_powerOffset = 0;
volatile bool lFwd = true;
volatile bool rFwd = true;
volatile bool full_stopped = true;
bool moving_straight = false;
int fwd_heading;
char current_command[20];

// cmd_vel vars
volatile bool cmd_vel_mode = false;

unsigned long last_laser_scan_ms = 0;
bool laser_scan_allowed = false;
int laser_scan_attempts = 2;

Servo head_pan_servo;
Servo head_tilt_servo;
unsigned long last_head_servos_move_ts = 0;

using fchassis_srv::FCommand;
using fchassis_srv::FTwistScan;
using fchassis_srv::FScannerSwitch;
using fchassis_srv::FScannerSetDirection;
using fchassis_srv::AngleRange;
using stuck_detector::Stuck;
ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
sensor_msgs::Range back_range_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mf_msg;
sensor_msgs::LaserScan laser_scan_msg;
fchassis_srv::mstate state_msg;
std_msgs::Int16 lwheel_msg;
std_msgs::Int16 rwheel_msg;

ros::Publisher pub_range( "sonar", &range_msg);
ros::Publisher pub_back_range( "back_sonar", &back_range_msg);
ros::Publisher pub_imu( "imu", &imu_msg);
ros::Publisher pub_laser_scan( "laser_scan", &laser_scan_msg);
ros::Publisher pub_mf( "mf", &mf_msg);
ros::Publisher pub_lwheel("lwheel", &lwheel_msg);
ros::Publisher pub_rwheel("rwheel", &rwheel_msg);
ros::Publisher pub_state( "state", &state_msg);

void stuckCb(const stuck_detector::Stuck &msg)
{
	if(msg.stuck) {
		move2release(60, 60);
		stop_after(10);
	}
}

ros::Subscriber<stuck_detector::Stuck> sub_stuck("stuck", &stuckCb);

void cmd_velCb(const geometry_msgs::Twist &msg)
{
	float left, right;
	float x = msg.linear.x; // m/s
	float th = msg.angular.z; // rad/s
	nh.loginfo(("l X:" + String(x) + ", a Z:" + String(th)).c_str());
	cmd_vel_mode = true;
	full_stopped = false;
	if(x == 0) {
		// Turn in place
		right = th * WHEEL_TRACK  * GEAR_REDUCTION / 2.0;
		left = -right;
	} else if(th == 0) {
		// Pure forward/backward motion
		left = right = x;
	} else {
		// Rotation about a point in space
		left = x - th * WHEEL_TRACK * GEAR_REDUCTION / 2;
		right = x + th * WHEEL_TRACK  * GEAR_REDUCTION / 2.0;
	}
//		nh.loginfo(("Left:" + String(left, 2)).c_str());
//		nh.loginfo(("Right:" + String(right, 2)).c_str());
	leftPID.TargetTicksPerFrame = round(left * TICKS_PER_METER / PID_RATE);
	rightPID.TargetTicksPerFrame = round(right * TICKS_PER_METER / PID_RATE);
	setLeftMotor(30, left > 0);
	setRightMotor(30, right > 0);
	
//		nh.loginfo(("Left ticks:" + String(leftPID.TargetTicksPerFrame)).c_str());
//		nh.loginfo(("Right ticks:" + String(rightPID.TargetTicksPerFrame)).c_str());
	stop_after(5);
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCb);

void command_callback(const FCommand::Request & req, FCommand::Response & res)
{
	nh.loginfo(("Cmd:" + String(req.mcommand)).c_str());
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

void twist_scan_callback(const FTwistScan::Request & req, FTwistScan::Response & res)
{
	const int start_angle = SONAR_PAN_ANGLE_MIN;
	const int end_angle = SONAR_PAN_ANGLE_MAX;
	const int step = 5;
	const int n_ranges = (end_angle - start_angle) / step;
	const int step_wait_pause = 30;
	static AngleRange ranges[n_ranges];
	res.ranges_length = n_ranges;
	res.ranges = ranges;
	unsigned long start_ms = millis();
	head_tilt_servo_move_to(req.tilt);
	for(int i=0; i<res.ranges_length; i++) {
		AngleRange r;
		r.angle = start_angle + i * step;
		head_pan_servo_move_to(r.angle);
		delay(step_wait_pause);
		r.range = getRange_HeadUltrasound(req.scan_attempts);
		res.ranges[i] = r;
	}
	head_pan_servo_move_to(sonarAngle);
	unsigned long end_ms = millis();
	if(start_ms < end_ms) {
		res.timeit = (start_ms < end_ms) ? end_ms - start_ms : ULONG_MAX - start_ms + end_ms;
	}
}

void scanner_switch_callback(const FScannerSwitch::Request & req, FScannerSwitch::Response & res)
{
	laser_scan_attempts = req.scan_attempts;
	laser_scan_allowed = req.scan_allowed;
	res.scan_allowed = laser_scan_allowed;
}

void scanner_set_direction_callback(const FScannerSetDirection::Request & req, FScannerSetDirection::Response & res)
{
	sonarAngle = req.direction + SONAR_PAN_CENTER;
	head_pan_servo_move_to(sonarAngle);
}

void fill_laser_scan()
{
	const int start_angle = SONAR_PAN_ANGLE_MIN;
	const int end_angle =SONAR_PAN_ANGLE_MAX;
	const int step = 5;
	const int n_ranges = (end_angle - start_angle) / step;
	const int step_wait_pause = 30;
	static float ranges[n_ranges];
	unsigned long start_ms = millis();
	laser_scan_msg.angle_min = -(end_angle - start_angle) / 2 * PI / 180;
	laser_scan_msg.angle_max = (end_angle - start_angle) / 2 * PI / 180;
	laser_scan_msg.angle_increment = step * PI / 180;
	laser_scan_msg.time_increment = step_wait_pause / 1000.;
	laser_scan_msg.scan_time = ((last_laser_scan_ms < start_ms) ? start_ms - last_laser_scan_ms : ULONG_MAX - last_laser_scan_ms + start_ms) / 1000.;
	laser_scan_msg.range_min = MIN_RANGE;
	laser_scan_msg.range_max = MAX_RANGE;
	laser_scan_msg.ranges_length = n_ranges;
	laser_scan_msg.ranges = ranges;
	head_tilt_servo_move_to(SONAR_TILT_CENTER);
	for(int i=0; i<n_ranges; i++) {
		int angle = start_angle + i * step;
		head_pan_servo_move_to(angle);
		delay(step_wait_pause);
		laser_scan_msg.ranges[i] = getRange_HeadUltrasound(laser_scan_attempts);
	}
	head_pan_servo_move_to(sonarAngle);
	last_laser_scan_ms = millis();
}


ros::ServiceServer<FCommand::Request, FCommand::Response> exec_command("exec_command",&command_callback);
ros::ServiceServer<FTwistScan::Request, FTwistScan::Response> twist_scan("twist_scan",&twist_scan_callback);
ros::ServiceServer<FScannerSwitch::Request, FScannerSwitch::Response> scanner_switch("scanner_switch",&scanner_switch_callback);
ros::ServiceServer<FScannerSetDirection::Request, FScannerSetDirection::Response> scanner_set_direction("scanner_set_direction",&scanner_set_direction_callback);

const char range_frameid[] = "/ultrasound";
const char back_range_frameid[] = "/back_ultrasound";
const char imu_frameid[] = "/imu";
const char base_frameid[] = "/base_link";

void lIntCB()
{
	lCounter += (lFwd) ? 1 : -1;
}

void rIntCB()
{
	rCounter += (rFwd) ? 1 : -1;
}

void head_pan_servo_move_to(int pos)
{
	if(!head_pan_servo.attached()) {
		head_pan_servo.attach(headPanServoPin);
	}
	if(pos < SONAR_PAN_ANGLE_MIN) pos = SONAR_PAN_ANGLE_MIN;
	if(pos > SONAR_PAN_ANGLE_MAX) pos = SONAR_PAN_ANGLE_MAX;
	head_pan_servo.write(pos);
	last_head_servos_move_ts = millis();
}

void head_tilt_servo_move_to(int pos)
{
	if(!head_tilt_servo.attached()) {
		head_tilt_servo.attach(headTiltServoPin);
	}
	if(pos < SONAR_TILT_ANGLE_MIN) pos = SONAR_TILT_ANGLE_MIN;
	if(pos > SONAR_TILT_ANGLE_MAX) pos = SONAR_TILT_ANGLE_MAX;
	head_tilt_servo.write(pos);
	last_head_servos_move_ts = millis();
}

void setup()
{

	// IR serial
	Serial3.begin(9600);

	pinMode(headTrigPin, OUTPUT);
	pinMode(headEchoPin, INPUT);
	pinMode(backTrigPin, OUTPUT);
	pinMode(backEchoPin, INPUT);

	pinMode(LEFT_MOTOR_POWER, OUTPUT);
	pinMode(LEFT_MOTOR_FWD, OUTPUT);
	pinMode(RIGHT_MOTOR_POWER, OUTPUT);
	pinMode(RIGHT_MOTOR_FWD, OUTPUT);

	pinMode(Eleft, INPUT);
	pinMode(Eright, INPUT);

	digitalWrite(Eleft, INPUT_PULLUP);
	digitalWrite(Eright, INPUT_PULLUP);

	head_pan_servo_move_to(SONAR_PAN_CENTER);
	head_tilt_servo_move_to(SONAR_PAN_CENTER);

	setup_compass();
	setup_accel();
	setup_gyro();
	setup_bmp085();

	attachInterrupt(digitalPinToInterrupt(Eleft), lIntCB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(Eright), rIntCB, CHANGE);

	nh.initNode();
	nh.advertise(pub_range);
	nh.advertise(pub_back_range);
	nh.advertise(pub_imu);
	nh.advertise(pub_laser_scan);
	nh.advertise(pub_mf);
	nh.advertise(pub_state);
	nh.advertise(pub_lwheel);
	nh.advertise(pub_rwheel);
	nh.advertiseService(exec_command);
	nh.advertiseService(twist_scan);
	nh.advertiseService(scanner_switch);
	nh.advertiseService(scanner_set_direction);
	nh.subscribe(sub_stuck);
	nh.subscribe(sub_cmd_vel);

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.header.frame_id = range_frameid;
	range_msg.field_of_view = 0.1; // fake
	range_msg.min_range = MIN_RANGE;
	range_msg.max_range = MAX_RANGE;

	back_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	back_range_msg.header.frame_id = back_range_frameid;
	back_range_msg.field_of_view = 0.1; // fake
	back_range_msg.min_range = MIN_RANGE;
	back_range_msg.max_range = MAX_RANGE;

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

	laser_scan_msg.header.frame_id = range_frameid;

	mf_msg.header.frame_id = imu_frameid;

	state_msg.header.frame_id = base_frameid;

	EventFuse::newFuse(100, INF_REPEAT, evHeadSonar);
	EventFuse::newFuse(100, INF_REPEAT, evBackSonar);
	EventFuse::newFuse(100, INF_REPEAT, evFixDir);

//	EventFuse::newFuse(100, INF_REPEAT, evMoveSonar);

	EventFuse::newFuse(3000, INF_REPEAT, evLaserScan);
//	EventFuse::newFuse(1000/PID_RATE, INF_REPEAT, evPIDupdate);
	EventFuse::newFuse(2000, INF_REPEAT, evHeadServoDetach);
	EventFuse::newFuse(5, INF_REPEAT, evIRcmd);

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
		range_msg.range = getRange_HeadUltrasound();
		range_msg.header.stamp = now;
		pub_range.publish(&range_msg);
		back_range_msg.range = getRange_BackUltrasound();
		back_range_msg.header.stamp = now;
		pub_range.publish(&back_range_msg);
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
		state_msg.pwroffset = pid_powerOffset;
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
