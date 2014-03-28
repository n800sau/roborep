// to get blinking led from camera

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/circular_buffer.hpp>

image_transport::Subscriber img_sub;
image_transport::Publisher img_pub;


#define MSGBUF_SIZE 24
boost::circular_buffer<sensor_msgs::ImageConstPtr> msgbuf(MSGBUF_SIZE);

void process_cb (const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	cv::Mat mmed;
	cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	cv::cvtColor(cv_ptr->image, mmed, CV_BGR2GRAY);
	cv::threshold(mmed, mmed, 240, 255 ,cv::THRESH_BINARY);
//	mmed.convertTo(mmed, CV_8U);
	cv_bridge::CvImage out_msg;
	out_msg.header   = msg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	out_msg.image    = mmed; // Your cv::Mat
	img_pub.publish(out_msg.toImageMsg());
/*	try
	{
		cv_ptr1 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
*/
/*	msgbuf.push_back(msg);
	if(msgbuf.size() == MSGBUF_SIZE) {
		cv::Mat msrc[MSGBUF_SIZE], mmed, covar, mean;
		cv_bridge::CvImageConstPtr cv_ptr;
		sensor_msgs::ImageConstPtr pmsg;
		const int sample_count = MSGBUF_SIZE-2;
		for(int i=0; i<sample_count; i++) {
			pmsg = msgbuf.front();
			cv_ptr = cv_bridge::toCvShare(pmsg, sensor_msgs::image_encodings::BGR8);
			cv::cvtColor(cv_ptr->image, mmed, CV_BGR2GRAY);
			cv::threshold(mmed, mmed, 127, 255 ,cv:;THRESH_BINARY);
//			mmed.convertTo(msrc[i], CV_64F);
			msgbuf.pop_front();
//			printf("w=%d,h=%d\n", msrc[i].cols, msrc[i].rows);
		}
		printf("HERE1\n");
		cv::calcCovarMatrix(msrc, sample_count, covar, mean, CV_COVAR_NORMAL, CV_64F);
		printf("HERE2\n");
//		cv_bridge::CvImage out_msg;
//		out_msg.header   = msg->header; // Same timestamp and tf frame as input image
//		out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
//		out_msg.image    = mean; // Your cv::Mat
//		img_pub.publish(out_msg.toImageMsg());
	}*/
/*		cv::Mat(msg->height, msg->width, CV_8UC1);
		cv_bridge::CvImageConstPtr cv_ptr1, cv_ptr2;
		sensor_msgs::ImageConstPtr p1, p2;
		p1 = msgbuf.front();
		cv_ptr1 = cv_bridge::toCvShare(p1, sensor_msgs::image_encodings::BGR8);
		msgbuf.pop_front();
		for(int i=0; i<MSGBUF_SIZE-2; i++) {
			p2 = msgbuf.front();
			msgbuf.pop_front();
			cv_ptr2 = cv_bridge::toCvShare(p2, sensor_msgs::image_encodings::BGR8);
			cv::Mat mdest;
			cv::absdiff(cv_ptr1->image, cv_ptr2->image, mdest);
		}
	}
*/	// Draw an example circle on the video stream
//	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
//		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
//	}

	// Output modified video stream
//	img_pub.publish(cv_ptr2->toImageMsg());

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
