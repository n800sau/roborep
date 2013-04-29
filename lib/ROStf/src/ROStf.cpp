#include "ROStf.h"
#include <unistd.h>
#include <math.h>
#include <syslog.h>

#include "ros/ros.h"
#include <sstream>

void ROStf_callback(const tf::tfMessageConstPtr &msg, ROStf *ths)
{
	ths->tf_message_received(msg);
}

ROStf::ROStf(int argc, char **argv):ReServant("rostf") {
	this->argc = argc;
	this->argv = argv;
}

bool ROStf::create_servant()
{
	bool rs = ReServant::create_servant();
	if(rs) {
		ros::init(argc, argv, "tfreader");
		ros::NodeHandle n;

//		ros::Subscriber sub = n.subscribe("tf", 1000, ROStf_callback);
		ros::Subscriber subLeft = n.subscribe<tf::tfMessage> ("tf", 1000, boost::bind(ROStf_callback, _1, this) );
	}
	return rs;
}

void ROStf::tf_message_received(const tf::tfMessageConstPtr &msg)
{
	ROS_INFO("received");
}

void ROStf::fill_json(json_t *js)
{
}

void ROStf::loop()
{
	json2redislist();
	ReServant::loop();
	ros::spinOnce();
}
