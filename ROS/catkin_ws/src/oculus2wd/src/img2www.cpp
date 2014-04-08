// to get blinking led from camera

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

std::string imgfname;
image_transport::Subscriber img_sub;

ros::Time last_stamp;
ros::Duration step;

bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  return true;
}

void saveimage_cb(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat dst;
	ros::Duration diff = msg->header.stamp - last_stamp;
	if(diff > step) {
		last_stamp = msg->header.stamp;
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		cv::resize(cv_ptr->image, dst, cv::Size(160,120));
		cv::cvtColor(dst, dst, CV_BGR2RGB);
		char text[50];
		time_t t = last_stamp.sec;
		std::strftime(text, sizeof(text), "%H:%M:%S", localtime(&t));
//		printf("%s\n", text);
		cv::putText(dst, text, cvPoint(10,10), CV_FONT_HERSHEY_PLAIN, 0.5, cvScalar(0, 0, 250), 1, CV_AA);
		unsigned pos = imgfname.find_last_of('.');
		std::string ext = imgfname.substr(pos+1);
		std::string tmpfname = imgfname + ".tmp." + ext;
		cv::imwrite(tmpfname.c_str(), dst);
		printf("%s\n", imgfname.c_str());
		rename(tmpfname.c_str(), imgfname.c_str());
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "image2www");
	ros::NodeHandle nh;
	std::string topic = nh.resolveName("image");
	printf("image=%s\n", topic.c_str());
	last_stamp = ros::Time::now();
	step = ros::Duration(1, 0);
	image_transport::ImageTransport it(nh);
	img_sub = it.subscribe(topic, 1, saveimage_cb);

	ros::NodeHandle local_nh("~");
	local_nh.param("filename", imgfname, std::string(std::string("/var/www/rgbframe/frame.png")));
	ros::ServiceServer save = local_nh.advertiseService ("save", service);

	ros::spin();
}
