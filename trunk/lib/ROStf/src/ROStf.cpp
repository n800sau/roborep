#include "ROStf.h"
#include <unistd.h>
#include <math.h>
#include <syslog.h>

#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <sstream>

void ROStf_callback(const tf::tfMessageConstPtr &msg, ROStf *ths)
{
	printf("Received\n");
	ths->tf_message_received(msg);
}

ROStf::ROStf(int argc, char **argv):ReServant("rostf") {
	this->argc = argc;
	this->argv = argv;
}

bool ROStf::create_servant()
{
	bool rs = ReServant::create_servant();
	printf("Created\n");
	if(rs) {
		printf("Init start\n");
		ros::init(argc, argv, "/tfreader");
		printf("Init end\n");
		ros::NodeHandle n;

		ros::Subscriber subLeft = n.subscribe<tf::tfMessage> ("/tf", 2000, boost::bind(ROStf_callback, _1, this) );
		printf("Subscribed\n");
	}
	return rs;
}

void ROStf::tf_message_received(const tf::tfMessageConstPtr &msg)
{
	ROS_INFO("received");
	for (unsigned int i = 0; i < msg->transforms.size(); i++)
	{
/*		tf::StampedTransform trans;
		transformStampedMsgToTF(msg->transforms[i], trans);
		try {
			std::map<std::string, std::string>* msg_header_map = msg->__connection_header.get();
			std::string authority;
			std::map<std::string, std::string>::iterator it = msg_header_map->find("callerid");
			if (it == msg_header_map->end()) {
				ROS_WARN("Message recieved without callerid");
			} else {
			}
		} catch (tf::TransformException& ex) {
			///\todo Use error reporting
			std::string temp = ex.what();
			ROS_ERROR("Failure: %s\n", temp.c_str());
		}*/
	}
}

bool ROStf::fill_json(json_t *js)
{
	return false;
}

void ROStf::loop()
{
	json2redislist();
	ReServant::loop();
	ros::spinOnce();
}
