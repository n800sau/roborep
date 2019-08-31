#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>

const int left_pwm = PB9;
const int left_in1 = PB12;
const int left_in2 = PB13;

const int right_pwm = PB8;
const int right_in1 = PB15;
const int right_in2 = PB14;

#define LED_BUILDING PC13

const int leftQeiAPin = PA15;
const int leftQeiBPin = PA12;

const int rightQeiAPin = PB4;
const int rightQeiBPin = PB3;


const int minPwr = 128;
//const int minPwr = 64;
const int maxPwr = 255;

// encoders
volatile int16_t leftCounts = 0, rightCounts = 0;
volatile int16_t oldLeftCounts = 0, oldRightCounts = 0;
// micros per encoder step
volatile unsigned long int leftElapsedTime = 0, rightElapsedTime = 0;
volatile unsigned long int leftOldElapsedTime = 0, rightOldElapsedTime = 0;

// rate
int req_left_rate = 0;
int req_right_rate = 0;

int leftPower = 0, oldLeftPower = 0;
int rightPower = 0, oldRightPower = 0;

//PID

//adjust these values to fit your own design
double Kp = 0.04;
double Kd = 0.03;
double Ki = 0.05;

double pid_req_left_rate = 0;
double pid_cur_left_rate, pid_left_power;
PID left_pid(&pid_cur_left_rate, &pid_left_power, &pid_req_left_rate, Kp, Ki, Kd, DIRECT);

double pid_req_right_rate = 0;
double pid_cur_right_rate, pid_right_power;
PID right_pid(&pid_cur_right_rate, &pid_right_power, &pid_req_right_rate, Kp, Ki, Kd, DIRECT);

ros::NodeHandle nh;

// pwr: -255 .. 255
void move_motor(int pin1, int pin2, int pin_pwm, int pwr)
{
	if(pwr == 0) {
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, LOW);
	} else if(pwr > 0) {
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, HIGH);
	} else {
		digitalWrite(pin1, HIGH);
		digitalWrite(pin2, LOW);
	}
	analogWrite(pin_pwm, abs(pwr));
}

std_msgs::Int16 l_ticks;
ros::Publisher l_ticks_pub("lwheel_ticks", &l_ticks); 
std_msgs::Int16 r_ticks;
ros::Publisher r_ticks_pub("rwheel_ticks", &r_ticks); 

std_msgs::Int16 l_rate;
ros::Publisher l_rate_pub("lwheel_rate", &l_rate); 
std_msgs::Int16 r_rate;
ros::Publisher r_rate_pub("rwheel_rate", &r_rate); 

std_msgs::Int16 l_pwr;
ros::Publisher l_pwr_pub("lwheel_pwr", &l_pwr); 
std_msgs::Int16 r_pwr;
ros::Publisher r_pwr_pub("rwheel_pwr", &r_pwr); 


void move_left()
{
	int l_p;
	if(req_left_rate == 0) {
		l_p = 0;
	} else {
		l_p = leftPower;
	}
	move_motor(left_in1, left_in2, left_pwm, l_p);
	if(oldLeftPower != l_p) {
		l_pwr.data = l_p;
		l_pwr_pub.publish(&l_pwr);
		oldLeftPower = l_p;
	}
}

void move_right()
{
	int r_p;
	if(req_right_rate == 0) {
		r_p = 0;
	} else {
		r_p = rightPower;
	}
	move_motor(right_in1, right_in2, right_pwm, r_p);
	if(oldRightPower != r_p) {
		r_pwr.data = r_p;
		r_pwr_pub.publish(&r_pwr);
		oldRightPower = r_p;
	}
}

void on_lwheel_rate(const std_msgs::Int16 &msg)
{
	if(req_left_rate != msg.data) {
		req_left_rate = msg.data;
		nh.loginfo((String("left_rate:") + String(req_left_rate)).c_str());
		leftElapsedTime = 0;
		// initial start
		pid_req_left_rate = abs(req_left_rate);
		move_left();
	}
}

void on_rwheel_rate(const std_msgs::Int16 &msg)
{
	if(req_right_rate != msg.data) {
		req_right_rate = msg.data;
		nh.loginfo((String("right_rate:") + String(req_right_rate)).c_str());
		rightElapsedTime = 0;
		// initial start
		pid_req_right_rate = abs(req_right_rate);
		move_right();
	}
}

void stop_cb(const std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	req_left_rate = req_right_rate = 0;
	move_left();
	move_right();
	nh.loginfo("stop motors");
}

ros::Subscriber<std_msgs::Int16> sub_lw_rate("lwheel_desired_rate", &on_lwheel_rate);
ros::Subscriber<std_msgs::Int16> sub_rw_rate("rwheel_desired_rate", &on_rwheel_rate);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> service_stop("stop", &stop_cb);

void leftQuickQEI()
{
	const int iv = 1;
	//a fast(ish) QEI function
	int state = 0;
	unsigned long int microTime;
	state = digitalRead(leftQeiAPin) << 1;
	state = state|digitalRead(leftQeiBPin);
	switch (state)
	{
		case 0:
		case 3:
			leftCounts -= iv;
			break;
		case 1:
		case 2:
			leftCounts += iv;
			break;
	}

	microTime = micros();
	leftElapsedTime = microTime-leftOldElapsedTime;
	leftOldElapsedTime = microTime;
	oldLeftCounts = leftCounts;
}

void rightQuickQEI()
{
	const int iv = -1;
	//a fast(ish) QEI function
	int state = 0;
	int microTime;
	state = digitalRead(rightQeiAPin) << 1;
	state = state|digitalRead(rightQeiBPin);
	switch (state)
	{
		case 0:
		case 3:
			rightCounts += iv;
			break;
		case 2:
			rightCounts -= iv;
			break;
	}
	microTime = micros();
	rightElapsedTime = microTime-rightOldElapsedTime;
	rightOldElapsedTime = microTime;
	oldRightCounts = rightCounts;
}

void setup()
{
	// Set up the built-in LED pin as an output:
	pinMode(LED_BUILDING, OUTPUT);
	pinMode(left_in1, OUTPUT);
	pinMode(left_in2, OUTPUT);
	pinMode(left_pwm, OUTPUT);
	pinMode(right_in1, OUTPUT);
	pinMode(right_in2, OUTPUT);
	pinMode(right_pwm, OUTPUT);
	pinMode(leftQeiAPin, INPUT_PULLUP);
	pinMode(rightQeiAPin, INPUT_PULLUP);
	pinMode(leftQeiBPin, INPUT_PULLUP);
	pinMode(rightQeiBPin, INPUT_PULLUP);

	//setup left PID
	left_pid.SetMode(AUTOMATIC);
	left_pid.SetSampleTime(5);
	left_pid.SetOutputLimits(minPwr, maxPwr);

	//setup right PID
	right_pid.SetMode(AUTOMATIC);
	right_pid.SetSampleTime(5);
	right_pid.SetOutputLimits(minPwr, maxPwr);

	attachInterrupt(leftQeiAPin, leftQuickQEI, CHANGE);
	attachInterrupt(rightQeiAPin, rightQuickQEI, CHANGE);
	//start the wheels
	leftElapsedTime = rightElapsedTime = micros();

	nh.initNode();
	nh.advertise(l_ticks_pub);
	nh.advertise(r_ticks_pub);
	nh.advertise(l_rate_pub);
	nh.advertise(r_rate_pub);
	nh.advertise(l_pwr_pub);
	nh.advertise(r_pwr_pub);
	nh.subscribe(sub_lw_rate);
	nh.subscribe(sub_rw_rate);
	nh.advertiseService(service_stop);
}


void ledSwitch()
{
	digitalWrite(LED_BUILDING,!digitalRead(LED_BUILDING));// Turn the LED from off to on, or on to off
}

void loop()
{

	int l_dir = (req_left_rate >= 0 ? 1 : -1);
	if(leftElapsedTime > 0) {
		pid_cur_left_rate = 1000000. / leftElapsedTime;
		if(left_pid.Compute()) {
			l_rate.data = pid_cur_left_rate * l_dir;
			l_rate_pub.publish(&l_rate);
			if(pid_left_power > 0) {
				leftPower = pid_left_power * l_dir;
				move_left();
			}
		}
		if(oldLeftCounts != leftCounts) {
			l_ticks.data = leftCounts;
			l_ticks_pub.publish(&l_ticks);
		}
	} else if(req_left_rate != 0) {
		// if req_rate != 0 motor must move
		if(abs(leftPower) < maxPwr) {
			leftPower += 10 * l_dir;
		}
		move_left();
	}

	int r_dir = (req_right_rate >= 0 ? 1 : -1);
	if(rightElapsedTime > 0) {
		pid_cur_right_rate = 1000000. / rightElapsedTime;
		if(right_pid.Compute()) {
			r_rate.data = pid_cur_right_rate;
			r_rate_pub.publish(&r_rate);
			if(pid_right_power > 0) {
				rightPower = pid_right_power * r_dir;
				move_right();
			}
		}
		if(oldRightCounts != rightCounts) {
			r_ticks.data = rightCounts;
			r_ticks_pub.publish(&r_ticks);
		}
	} else if(req_right_rate != 0) {
		// if req_rate != 0 motor must move
		if(abs(rightPower) < maxPwr) {
			rightPower += 10 * r_dir;
		}
		move_right();
	}

	nh.spinOnce();

}
