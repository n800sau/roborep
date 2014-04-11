// to get blinking led from camera

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include"skin_detector.hpp"

std::string imgfname, clrfname, skinfname, circlesfname;
image_transport::Subscriber img_sub;

ros::Time last_stamp;
ros::Duration step;

bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  return true;
}

void write_img(const cv::Mat img, std::string fname)
{
	cv::Mat c_img = img.clone();
	char text[50];
	time_t t = last_stamp.sec;
	std::strftime(text, sizeof(text), "%H:%M:%S", localtime(&t));
//	printf("%s\n", text);
	cv::putText(c_img, text, cvPoint(10,10), CV_FONT_HERSHEY_PLAIN, 0.5, cvScalar(0, 0, 250), 1, CV_AA);
	unsigned pos = fname.find_last_of('.');
	std::string ext = fname.substr(pos+1);
	std::string tmpfname = fname + ".tmp." + ext;
	cv::imwrite(tmpfname.c_str(), c_img);
	rename(tmpfname.c_str(), fname.c_str());
}

void saveimage_cb(const sensor_msgs::ImageConstPtr& msg)
{
	SkinDetector mySkinDetector;
	cv::Mat dst, imgHSV, mask, skinMat;
	ros::Duration diff = msg->header.stamp - last_stamp;
	if(diff > step) {
		last_stamp = msg->header.stamp;
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		cv::resize(cv_ptr->image, dst, cv::Size(160,120));
		dst.copyTo(imgHSV);
		cv::cvtColor(dst, dst, CV_BGR2RGB);
		write_img(dst, imgfname);

		cv::GaussianBlur( imgHSV, imgHSV, cv::Size( 3, 3 ), 0, 0 );
		cv::cvtColor(imgHSV, imgHSV, CV_BGR2HSV);
//		cv::inRange(imgHSV, cv::Scalar(170,160,60), cv::Scalar(180,255,255), mask);
		cv::inRange(imgHSV, cv::Scalar(90,0,0), cv::Scalar(180,255,255), mask);
//		cv::inRange(imgHSV, cv::Scalar(0,0,0), cv::Scalar(255,255,255), mask);
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/20, 200, 100, 0, 0 );
		for( size_t i = 0; i < circles.size(); i++ )
		{
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// draw the circle center
			cv::circle( dst, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
			// draw the circle outline
			cv::circle( dst, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
		}
		write_img(dst, circlesfname);
		int morph_size = 10;
//		cv::Mat element = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
//		cv::morphologyEx( mask, mask, cv::MORPH_OPEN, element);
//		cv::GaussianBlur( mask, mask, cv::Size( 3, 3 ), 0, 0 );
		cv::cvtColor(mask, mask, CV_GRAY2RGB);
		write_img(mask, clrfname);
//		write_img(imgHSV, clrfname);
		skinMat= mySkinDetector.getSkin(dst);
		write_img(skinMat, skinfname);

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
	local_nh.param("filename_raw", imgfname, std::string(std::string("/var/www/rgbframe/frame.png")));
	local_nh.param("filename_color", clrfname, std::string(std::string("/var/www/rgbframe/frame_color.png")));
	local_nh.param("filename_skin", skinfname, std::string(std::string("/var/www/rgbframe/frame_skin.png")));
	local_nh.param("filename_circles", circlesfname, std::string(std::string("/var/www/rgbframe/frame_circles.png")));
	ros::ServiceServer save = local_nh.advertiseService ("save", service);

	ros::spin();
}
