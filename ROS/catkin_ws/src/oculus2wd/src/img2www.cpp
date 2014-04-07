// to get blinking led from camera

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


const char imgfname[] = "/var/www/rgbframe/frame.png";
image_transport::Subscriber img_sub;

ros::Time last_stamp;
ros::Duration step;

void saveimage_cb(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat dst;
	ros::Duration diff = msg->header.stamp - last_stamp;
	if(diff > step) {
		last_stamp = msg->header.stamp;
//		printf("Tick\n");
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
		cv::resize(cv_ptr->image, dst, cv::Size(160,120));
		char text[50];
		time_t t = last_stamp.sec;
		std::strftime(text, sizeof(text), "%H:%M:%S", localtime(&t));
//		printf("%s\n", text);
		cv::putText(dst, text, cvPoint(10,10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 100, 0), 1, CV_AA);
		cv::imwrite(imgfname, dst);
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "image2www");
	ros::NodeHandle nh;
	last_stamp = ros::Time::now();
	step = ros::Duration(1, 0);
	image_transport::ImageTransport it(nh);
	img_sub = it.subscribe("/camera/rgb/image_raw", 1, saveimage_cb);
	ros::spin();
}
