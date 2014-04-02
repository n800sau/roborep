// to get blinking led from camera

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp> //This is where actual SURF and SIFT algorithm is located
#include <boost/circular_buffer.hpp>

image_transport::Subscriber img_sub;
image_transport::Publisher img_pub;


#define MSGBUF_SIZE 2
boost::circular_buffer<sensor_msgs::ImageConstPtr> msgbuf(MSGBUF_SIZE);

void process_cb (const sensor_msgs::ImageConstPtr& msg)
{
/*	cv_bridge::CvImageConstPtr cv_ptr;
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
*/
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
	int x, y;
	msgbuf.push_back(msg);
	printf("tick %d\n", msg->header.seq);
	if(msgbuf.size() >= MSGBUF_SIZE) {
		cv::Mat msrc[MSGBUF_SIZE-1], mold, mnew, mmask, mout;
		cv_bridge::CvImageConstPtr cv_ptr;
		sensor_msgs::ImageConstPtr pmsg;
		for(int i=0; i<MSGBUF_SIZE; i++) {
			pmsg = msgbuf.front();
			cv_ptr = cv_bridge::toCvShare(pmsg, sensor_msgs::image_encodings::BGR8);
			cv::cvtColor(cv_ptr->image, mnew, CV_BGR2GRAY);
//			mnew.convertTo(mnew, CV_32F);
			msgbuf.pop_front();
			if(i>0) {
//				mmask = mnew;
//				printf("types:%d,%d\n", mold.type(), mnew.type());
//				absdiff(mold, mnew, mmask);
				mmask = mnew - mold;
//				mmask.convertTo(mmask, CV_8UC1);
//				mmed.convertTo(oldm, CV_64F);
//				cv_ptr->image.convertTo(newm, CV_16F);
//				cvSub(&oldm, &newm, &msrc[i-1]);
				cv::threshold(mmask, mmask, 100, 255, cv::THRESH_BINARY);

				//reduce area
				//printf("-------------------------------------------------------------------------\n");
				bool found = false;
				std::set<int> xset;
				std::set<int> yset;
				cv::Point pmin(mmask.size().width, mmask.size().height), pmax(0, 0);
				for(y=0; y<mmask.size().height; y++) {
					uchar* p = mmask.ptr(y);
					for(x=0; x<mmask.size().width; x++) {
						if(p[x] > 0) {
							found = true;
							xset.insert(x);
							yset.insert(y);
							if(pmax.x < x) {
								pmax.x = x;
							}
							if(pmax.y < y) {
								pmax.y = y;
							}
							if(pmin.x > x) {
								pmin.x = x;
							}
							if(pmin.y > y) {
								pmin.y = y;
							}
						}
					}
				}
				if(found && (pmax.x - pmin.x) < 50 && (pmax.y - pmin.y) < 50) {
//					printf("found\n");
					std::vector<int> xlist;
					xlist.push_back(pmin.x);
					xlist.push_back(pmax.x);
					std::vector<int> ylist;
					ylist.push_back(pmin.y);
					ylist.push_back(pmax.y);
					//separate objects
					for(x=pmin.x; y<pmax.x; x++) {
						if(xset.find(x) == xset.end()) {
							//black line
							xlist.push_back(x);
						}
					}
					for(y=pmin.y; y<pmax.y; y++) {
						if(yset.find(y) == yset.end()) {
							//black line
							ylist.push_back(y);
						}
					}
					std::sort(xlist.begin(), xlist.end());
					std::sort(ylist.begin(), ylist.end());
					printf("%d:nobjects=%d\n", msg->header.seq, (xlist.size()-1) * (ylist.size()-1));
//					printf("%d,%d - %d,%d\n", pmin.x, pmin.y, pmax.x, pmax.y);
//				cv::cvtColor(mmask, mout, CV_GRAY2RGB);

//				int minHessian = 500;
//				cv::SurfFeatureDetector detector(minHessian);
//				std::vector<cv::KeyPoint> keypoints;
//				detector.detect(mmask, keypoints);
//				for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it)
//					printf("%.0f,%.0f\n", it->pt.x, it->pt.y);
					mout = cv_ptr->image;
					mout.setTo(cv::Scalar(0, 0, 0));
//				cv::drawKeypoints(mout, keypoints, mout, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

					cv::rectangle(mout, pmin, pmax, cv::Scalar::all(-1), 1, 8, 0);
//				contours,hier = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

//				std::vector<std::vector<cv::Point> > squares;
//				cv::find_squares(mmask, squares);
//				cv::draw_squares(mmask, squares);
					cv_bridge::CvImage out_msg;
					out_msg.header   = msg->header; // Same timestamp and tf frame as input image
//				out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
//				out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
					out_msg.encoding = pmsg->encoding;
					out_msg.image    = mout; // Your cv::Mat
					img_pub.publish(out_msg.toImageMsg());
				}
			}
			mold = mnew.clone();
//			mmed.convertTo(msrc[i], CV_64F);
//			printf("w=%d,h=%d\n", msrc[i].cols, msrc[i].rows);
		}
//		cv::calcCovarMatrix(msrc, sample_count, covar, mean, CV_COVAR_NORMAL, CV_64F);
	}
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
