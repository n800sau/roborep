#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

image_transport::Subscriber img_sub;
image_transport::Publisher img_pub;


void process_cb (const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw an example circle on the video stream
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	}

	// Output modified video stream
	img_pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "image_signal");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	img_sub = it.subscribe("/camera/rgb/image_raw", 3, process_cb);
	img_pub = it.advertise("/oculus2wd/signal_image", 1);
	ros::spin();
}
