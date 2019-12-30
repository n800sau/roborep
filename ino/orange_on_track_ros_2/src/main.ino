#include "encoders.h"
#include "motion.h"
#include "ros_handle.h"

#include <ros/time.h>
#include <ochassis_msgs/odom.h>
#include <ochassis_msgs/vel.h>
#include <ochassis_msgs/Move.h>
#include <ochassis_msgs/light.h>

// position
double x = 0.0;
double y = 0.0;
// angle
double th = 0;

const char base_link_id[] = "base_link";
const char odom_id[] = "odom";

ros::NodeHandle nh;

ochassis_msgs::odom odom;
ros::Publisher odom_pub("odom_s", &odom); 

unsigned long current_time = millis();
unsigned long last_time = current_time;

const int LIGHT_PIN = 48;

void move_cb(const ochassis_msgs::MoveRequest & req, ochassis_msgs::MoveResponse & res)
{
	float x = req.linear_x; // m/s
	float th = req.angular_z; // rad/s
	nh.loginfo(("move cmd X:" + String(x) + ", TH:" + String(th)).c_str());
	motion::set_motion(x, th);
	res.left = motion::left;
	res.right = motion::right;
}

void vel_sCb(const ochassis_msgs::vel &msg)
{
	float x = msg.linear_x; // m/s
	float th = msg.angular_z; // rad/s
	nh.loginfo(("motion X:" + String(x) + ", TH:" + String(th)).c_str());
	motion::set_motion(x, th);
}

void light_sCb(const ochassis_msgs::light &msg)
{
	digitalWrite(LIGHT_PIN, (msg.data) ? HIGH : LOW);
}

ros::Subscriber<ochassis_msgs::vel> sub_vel_s("vel_s", &vel_sCb);
ros::Subscriber<ochassis_msgs::light> sub_light("light", &light_sCb);
ros::ServiceServer<ochassis_msgs::MoveRequest, ochassis_msgs::MoveResponse> service_move("move", &move_cb);

void setup()
{
	pinMode(LIGHT_PIN, OUTPUT);
	encoders::setup();
	motion::setup();
	nh.initNode();
	nh.advertise(odom_pub);
	nh.subscribe(sub_vel_s);
	nh.subscribe(sub_light);
	nh.advertiseService(service_move);
}

void loop()
{
	if (nh.connected()) {
		current_time = millis();
		double dt = current_time - last_time;
		double ld = encoders::count2dist(encoders::lCounter);   // left wheel linear distance
		double rd = encoders::count2dist(encoders::rCounter);  // right wheel linear distance
		double vlm = ld / dt;								// left wheel linear velocity
		double vrm = rd / dt;							   // right wheel linear velocity
		double vx = (vlm + vrm) / 2.;						// base center forward velocity
		double vy = 0;
		double th = (encoders::rCounter - encoders::lCounter) / 45. * PI;
		double vth = th / dt;
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		x += delta_x;
		y += delta_y;
		last_time = current_time;

		// odometry
		odom.header.stamp = nh.now();
		odom.header.frame_id = odom_id;

		//set the position
		odom.x = x;
		odom.y = y;
		odom.th = th;

		//set the velocity
		odom.child_frame_id = base_link_id;
		odom.vx = vx;
		odom.vy = vy;
		odom.vth = vth;

		//publish the message
		odom_pub.publish(&odom);
		motion::tick();
		if(floor(motion::lOut) != 0 || floor(motion::rOut) != 0) {
//			nh.loginfo(("motion lOut:" + String(motion::lOut) + ", rOut:" + String(motion::rOut)).c_str());
		}
	} else {
		motion::stop();
	}
	nh.spinOnce();
	evIRcmd();
	// Loop exproximativly at 1Hz
	delay(200);
}
