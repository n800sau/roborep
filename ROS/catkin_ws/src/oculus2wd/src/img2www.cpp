// to get blinking led from camera

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

#include"skin_detector.hpp"

std::string imgfname, clrfname, skinfname, circlesfname, dispfname, motionfname, segmentobjectfname;
image_transport::Subscriber img_sub, mimg_sub, soimg_sub;
ros::Subscriber disp_sub;

cv::Mat_<cv::Vec3b> disparity_color;

ros::Time last_stamp;
ros::Duration step;

unsigned char colormap[768] = {
	150, 150, 150,
	107, 0, 12,
	106, 0, 18,
	105, 0, 24,
	103, 0, 30,
	102, 0, 36,
	101, 0, 42,
	99, 0, 48,
	98, 0, 54,
	97, 0, 60,
	96, 0, 66,
	94, 0, 72,
	93, 0, 78,
	92, 0, 84,
	91, 0, 90,
	89, 0, 96,
	88, 0, 102,
	87, 0, 108,
	85, 0, 114,
	84, 0, 120,
	83, 0, 126,
	82, 0, 131,
	80, 0, 137,
	79, 0, 143,
	78, 0, 149,
	77, 0, 155,
	75, 0, 161,
	74, 0, 167,
	73, 0, 173,
	71, 0, 179,
	70, 0, 185,
	69, 0, 191,
	68, 0, 197,
	66, 0, 203,
	65, 0, 209,
	64, 0, 215,
	62, 0, 221,
	61, 0, 227,
	60, 0, 233,
	59, 0, 239,
	57, 0, 245,
	56, 0, 251,
	55, 0, 255,
	54, 0, 255,
	52, 0, 255,
	51, 0, 255,
	50, 0, 255,
	48, 0, 255,
	47, 0, 255,
	46, 0, 255,
	45, 0, 255,
	43, 0, 255,
	42, 0, 255,
	41, 0, 255,
	40, 0, 255,
	38, 0, 255,
	37, 0, 255,
	36, 0, 255,
	34, 0, 255,
	33, 0, 255,
	32, 0, 255,
	31, 0, 255,
	29, 0, 255,
	28, 0, 255,
	27, 0, 255,
	26, 0, 255,
	24, 0, 255,
	23, 0, 255,
	22, 0, 255,
	20, 0, 255,
	19, 0, 255,
	18, 0, 255,
	17, 0, 255,
	15, 0, 255,
	14, 0, 255,
	13, 0, 255,
	11, 0, 255,
	10, 0, 255,
	9, 0, 255,
	8, 0, 255,
	6, 0, 255,
	5, 0, 255,
	4, 0, 255,
	3, 0, 255,
	1, 0, 255,
	0, 4, 255,
	0, 10, 255,
	0, 16, 255,
	0, 22, 255,
	0, 28, 255,
	0, 34, 255,
	0, 40, 255,
	0, 46, 255,
	0, 52, 255,
	0, 58, 255,
	0, 64, 255,
	0, 70, 255,
	0, 76, 255,
	0, 82, 255,
	0, 88, 255,
	0, 94, 255,
	0, 100, 255,
	0, 106, 255,
	0, 112, 255,
	0, 118, 255,
	0, 124, 255,
	0, 129, 255,
	0, 135, 255,
	0, 141, 255,
	0, 147, 255,
	0, 153, 255,
	0, 159, 255,
	0, 165, 255,
	0, 171, 255,
	0, 177, 255,
	0, 183, 255,
	0, 189, 255,
	0, 195, 255,
	0, 201, 255,
	0, 207, 255,
	0, 213, 255,
	0, 219, 255,
	0, 225, 255,
	0, 231, 255,
	0, 237, 255,
	0, 243, 255,
	0, 249, 255,
	0, 255, 255,
	0, 255, 249,
	0, 255, 243,
	0, 255, 237,
	0, 255, 231,
	0, 255, 225,
	0, 255, 219,
	0, 255, 213,
	0, 255, 207,
	0, 255, 201,
	0, 255, 195,
	0, 255, 189,
	0, 255, 183,
	0, 255, 177,
	0, 255, 171,
	0, 255, 165,
	0, 255, 159,
	0, 255, 153,
	0, 255, 147,
	0, 255, 141,
	0, 255, 135,
	0, 255, 129,
	0, 255, 124,
	0, 255, 118,
	0, 255, 112,
	0, 255, 106,
	0, 255, 100,
	0, 255, 94,
	0, 255, 88,
	0, 255, 82,
	0, 255, 76,
	0, 255, 70,
	0, 255, 64,
	0, 255, 58,
	0, 255, 52,
	0, 255, 46,
	0, 255, 40,
	0, 255, 34,
	0, 255, 28,
	0, 255, 22,
	0, 255, 16,
	0, 255, 10,
	0, 255, 4,
	2, 255, 0,
	8, 255, 0,
	14, 255, 0,
	20, 255, 0,
	26, 255, 0,
	32, 255, 0,
	38, 255, 0,
	44, 255, 0,
	50, 255, 0,
	56, 255, 0,
	62, 255, 0,
	68, 255, 0,
	74, 255, 0,
	80, 255, 0,
	86, 255, 0,
	92, 255, 0,
	98, 255, 0,
	104, 255, 0,
	110, 255, 0,
	116, 255, 0,
	122, 255, 0,
	128, 255, 0,
	133, 255, 0,
	139, 255, 0,
	145, 255, 0,
	151, 255, 0,
	157, 255, 0,
	163, 255, 0,
	169, 255, 0,
	175, 255, 0,
	181, 255, 0,
	187, 255, 0,
	193, 255, 0,
	199, 255, 0,
	205, 255, 0,
	211, 255, 0,
	217, 255, 0,
	223, 255, 0,
	229, 255, 0,
	235, 255, 0,
	241, 255, 0,
	247, 255, 0,
	253, 255, 0,
	255, 251, 0,
	255, 245, 0,
	255, 239, 0,
	255, 233, 0,
	255, 227, 0,
	255, 221, 0,
	255, 215, 0,
	255, 209, 0,
	255, 203, 0,
	255, 197, 0,
	255, 191, 0,
	255, 185, 0,
	255, 179, 0,
	255, 173, 0,
	255, 167, 0,
	255, 161, 0,
	255, 155, 0,
	255, 149, 0,
	255, 143, 0,
	255, 137, 0,
	255, 131, 0,
	255, 126, 0,
	255, 120, 0,
	255, 114, 0,
	255, 108, 0,
	255, 102, 0,
	255, 96, 0,
	255, 90, 0,
	255, 84, 0,
	255, 78, 0,
	255, 72, 0,
	255, 66, 0,
	255, 60, 0,
	255, 54, 0,
	255, 48, 0,
	255, 42, 0,
	255, 36, 0,
	255, 30, 0,
	255, 24, 0,
	255, 18, 0,
	255, 12, 0,
	255,	6, 0,
	255,	0, 0,
	};


bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	return true;
}

void write_img(const cv::Mat img, std::string fname, cv::Scalar clr=cvScalar(0, 0, 250))
{
	cv::Mat c_img = img.clone();
	char text[50];
	time_t t = last_stamp.sec;
	std::strftime(text, sizeof(text), "%H:%M:%S", localtime(&t));
//	printf("%s\n", text);
	cv::putText(c_img, text, cvPoint(10,10), CV_FONT_HERSHEY_SIMPLEX, 0.3, clr, 1, CV_AA);
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
			cv::circle( dst, center, 3, cvScalar(0,255,0), -1, 8, 0 );
			// draw the circle outline
			cv::circle( dst, center, radius, cvScalar(0,0,255), 3, 8, 0 );
		}

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(mask, lines, 1, CV_PI/180, 50, 50, 10 );
		for( size_t i = 0; i < lines.size(); i++ )
		{
			cv::Vec4i l = lines[i];
			cv::line( dst, cvPoint(l[0], l[1]), cvPoint(l[2], l[3]), cvScalar(0,0,255), 1, CV_AA);
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

void savemotion_cb(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat dst;
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	cv::resize(cv_ptr->image, dst, cv::Size(160,120));
	cv::cvtColor(dst, dst, CV_BGR2RGB);
	write_img(dst, motionfname);
}

void savesegmentobject_cb(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat dst;
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	cv::resize(cv_ptr->image, dst, cv::Size(160,120));
	cv::cvtColor(dst, dst, CV_BGR2RGB);
	write_img(dst, segmentobjectfname);
}

void savedisparityimage_cb(const stereo_msgs::DisparityImageConstPtr& msg)
{
	// Check for common errors in input
	if (msg->min_disparity == 0.0 && msg->max_disparity == 0.0)
	{
		ROS_ERROR("Disparity image fields min_disparity and max_disparity are not set");
		return;
	}
	if (msg->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
	{
		ROS_ERROR("Disparity image must be 32-bit floating point "
			"(encoding '32FC1'), but has encoding '%s'", msg->image.encoding.c_str());
		return;
	}

	// Colormap and display the disparity image
	float min_disparity = msg->min_disparity;
	float max_disparity = msg->max_disparity;
	float multiplier = 255.0f / (max_disparity - min_disparity);

	const cv::Mat_<float> dmat(msg->image.height, msg->image.width,
														 (float*)&msg->image.data[0], msg->image.step);
	disparity_color.create(msg->image.height, msg->image.width);
		
	for (int row = 0; row < disparity_color.rows; ++row) {
		const float* d = dmat[row];
		for (int col = 0; col < disparity_color.cols; ++col) {
			int index = (d[col] - min_disparity) * multiplier + 0.5;
			index = std::min(255, std::max(0, index));
			// Fill as BGR
			disparity_color(row, col)[2] = colormap[3*index + 0];
			disparity_color(row, col)[1] = colormap[3*index + 1];
			disparity_color(row, col)[0] = colormap[3*index + 2];
		}
	}

	cv::resize(disparity_color, disparity_color, cv::Size(160,120));
	write_img(disparity_color, dispfname, cvScalar(0, 0, 0));

}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "image2www");
	ros::NodeHandle nh;
	std::string topic1 = nh.resolveName("image");
	std::string topic2 = nh.resolveName("disparity");
	std::string topic3 = nh.resolveName("image_motion");
	std::string topic4 = nh.resolveName("image_segment_object");
//	printf("image=%s\n", topic1.c_str());
	last_stamp = ros::Time::now();
	step = ros::Duration(1, 0);
	image_transport::ImageTransport it(nh);
	img_sub = it.subscribe(topic1, 100, saveimage_cb);
	disp_sub = nh.subscribe<stereo_msgs::DisparityImage>(topic2, 100, savedisparityimage_cb, ros::TransportHints().unreliable());
	mimg_sub = it.subscribe(topic3, 100, savemotion_cb);
	soimg_sub = it.subscribe(topic4, 100, savesegmentobject_cb);

	ros::NodeHandle local_nh("~");
	local_nh.param("filename_raw", imgfname, std::string(std::string("/var/www/rgbframe/frame.png")));
	local_nh.param("filename_color", clrfname, std::string(std::string("/var/www/rgbframe/frame_color.png")));
	local_nh.param("filename_skin", skinfname, std::string(std::string("/var/www/rgbframe/frame_skin.png")));
	local_nh.param("filename_circles", circlesfname, std::string(std::string("/var/www/rgbframe/frame_circles.png")));
	local_nh.param("filename_disparity", dispfname, std::string(std::string("/var/www/rgbframe/frame_disparity.png")));
	local_nh.param("filename_motion", motionfname, std::string(std::string("/var/www/rgbframe/frame_motion.png")));
	local_nh.param("filename_segment_object", segmentobjectfname, std::string(std::string("/var/www/rgbframe/frame_segment_object.png")));
	ros::ServiceServer save = local_nh.advertiseService ("save", service);
//	ros::AsyncSpinner spinner(2);
//	spinner.start();

//	ros::Rate r(5);
//	while (ros::ok())
//	{
//		ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "].");
//		r.sleep();
//	}
	ros::spin();
}
