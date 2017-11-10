#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>

const int left_pwm = PB8;
const int left_in1 = PB15;
const int left_in2 = PB14;

const int right_pwm = PB9;
const int right_in3 = PB12;
const int right_in4 = PB13;


#define LED_BUILDING PC13

const int leftQeiAPin = PB3;
const int leftQeiBPin = PB4;

const int rightQeiAPin = PA15;
const int rightQeiBPin = PA12;


const int minPwr = 128;
const int maxPwr = 255;
volatile int16_t leftCounts = 0, rightCounts = 0;
volatile int16_t oldLeftCounts = 0, oldRightCounts = 0;
// micros per encoder step
volatile unsigned long int leftElapsedTime = 0, rightElapsedTime = 0;
volatile unsigned long int leftOldElapsedTime = 0, rightOldElapsedTime = 0;

bool cur_left_fwd = true;
bool cur_right_fwd = true;

int leftPower = 0;
int rightPower = 0;

bool stop_mode = true;

//PID

//adjust these values to fit your own design
double Kp = 20;
double Kd = 3;
double Ki = 10;

bool left_fwd = true;
double left_rate = 0;
double left_cur_rate, left_power;
PID left_pid(&left_cur_rate, &left_power, &left_rate, Kp, Ki, Kd, DIRECT);

bool right_fwd = true;
double right_rate = 0;
double right_cur_rate, right_power;
PID right_pid(&right_cur_rate, &right_power, &right_rate, Kp, Ki, Kd, DIRECT);

bool st_changed = false;

ros::NodeHandle nh;

// pwr: -255 .. 255
void move_motor(int pin1, int pin2, int pin_pwm, int pwr, bool fwd)
{
	if(fwd) {
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, HIGH);
	} else {
		digitalWrite(pin1, HIGH);
		digitalWrite(pin2, LOW);
	}
	analogWrite(pin_pwm, pwr);
}

void move()
{
	if(stop_mode) {
		move_motor(left_in1, left_in2, left_pwm, 0, true);
		move_motor(right_in3, right_in4, right_pwm, 0, true);
	} else {
		move_motor(left_in1, left_in2, left_pwm, leftPower, left_fwd);
		move_motor(right_in3, right_in4, right_pwm, rightPower, right_fwd);
	}
}

void on_lwheel_rate(const std_msgs::Int16 &msg)
{
	if(msg.data != 0) {
		left_fwd = msg.data >= 0;
		left_rate = msg.data;
		stop_mode = false;
		if(leftPower == 0) {
			// set initial power
			leftPower = left_fwd ? maxPwr : -maxPwr;
			move();
		}
	}
	nh.loginfo((String("left_rate:") + String(left_rate) + "," + String((left_fwd) ? "forward" : "back")).c_str());
}

void on_rwheel_rate(const std_msgs::Int16 &msg)
{
	if(msg.data != 0) {
		right_fwd = msg.data >= 0;
		right_rate = msg.data;
		stop_mode = false;
		if(rightPower == 0) {
			// set initial power
			rightPower = right_fwd ? maxPwr : -maxPwr;
			move();
		}
	}
	nh.loginfo((String("right_rate:") + String(right_rate) + "," + String((right_fwd) ? "forward" : "back")).c_str());
}

void stop_cb(const std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	stop_mode = true;
	nh.loginfo("stop mode set");
}

ros::Subscriber<std_msgs::Int16> sub_lw_rate("lwheel_desired_rate", &on_lwheel_rate);
ros::Subscriber<std_msgs::Int16> sub_rw_rate("rwheel_desired_rate", &on_rwheel_rate);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> service_stop("stop", &stop_cb);

std_msgs::Int16 l_ticks;
ros::Publisher l_ticks_pub("lwheel_ticks", &l_ticks); 
std_msgs::Int16 r_ticks;
ros::Publisher r_ticks_pub("rwheel_ticks", &r_ticks); 

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
	cur_left_fwd = oldLeftCounts - leftCounts;
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
	cur_right_fwd = oldRightCounts - rightCounts;
	oldRightCounts = rightCounts;
}

void setup()
{
	// Set up the built-in LED pin as an output:
	pinMode(LED_BUILDING, OUTPUT);
	pinMode(left_in1, OUTPUT);
	pinMode(left_in2, OUTPUT);
	pinMode(left_pwm, OUTPUT);
	pinMode(right_in3, OUTPUT);
	pinMode(right_in4, OUTPUT);
	pinMode(right_pwm, OUTPUT);
	pinMode(leftQeiAPin, INPUT_PULLUP);
	pinMode(rightQeiAPin, INPUT_PULLUP);
	pinMode(leftQeiBPin, INPUT_PULLUP);
	pinMode(rightQeiBPin, INPUT_PULLUP);

	//setup left PID
	left_pid.SetMode(AUTOMATIC);
	left_pid.SetSampleTime(10);
	left_pid.SetOutputLimits(minPwr, maxPwr);

	//setup right PID
	right_pid.SetMode(AUTOMATIC);
	right_pid.SetSampleTime(10);
	right_pid.SetOutputLimits(minPwr, maxPwr);

	attachInterrupt(leftQeiAPin, leftQuickQEI, CHANGE);
	attachInterrupt(rightQeiAPin, rightQuickQEI, CHANGE);
	//start the wheels
	leftElapsedTime = rightElapsedTime = micros();

	nh.initNode();
	nh.advertise(l_ticks_pub);
	nh.advertise(r_ticks_pub);
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
//	if (nh.connected()) {

		if(stop_mode) {
			// in move it stops
			move();
		} else {

			left_cur_rate = leftElapsedTime > 0 ? ((cur_left_fwd ? 1000000. : -1000000.) / leftElapsedTime) : 0;
			right_cur_rate = rightElapsedTime > 0 ? ((cur_right_fwd ? 1000000. : -1000000.) / rightElapsedTime) : 0;

			if(left_rate == 0) {
				st_changed = leftPower != 0;
				leftPower = 0;
			} else if(left_pid.Compute()) { 
				st_changed = true;
				leftPower = left_power;
			}

			if(right_rate == 0) {
				st_changed = rightPower != 0;
				rightPower = 0;
			} else if(right_pid.Compute()) {
				st_changed = true;
				rightPower = right_power;
			}
			if(st_changed) {
				move();
			}

		}

		if(oldLeftCounts != leftCounts) {
			l_ticks.data = leftCounts;
			l_ticks_pub.publish(&l_ticks);
		}
		if(oldRightCounts != rightCounts) {
			r_ticks.data = rightCounts;
			r_ticks_pub.publish(&r_ticks);
		}

		nh.spinOnce();

//	} else {
//		delay(1);
//	}

}

