#include <boost/version.hpp> 
#if ((BOOST_VERSION / 100) % 1000) >= 53 
#include <boost/thread/lock_guard.hpp> 
#endif 

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

#define MSGBUF_SIZE 2

namespace image_motion_nd
{

	class ImageMotionNd : public nodelet::Nodelet
	{
		public:
			ImageMotionNd() : msgbuf(MSGBUF_SIZE) {}

		private:
			virtual void onInit()
			{
				ros::NodeHandle &nh = getNodeHandle();
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				it.reset(new image_transport::ImageTransport(nh));
				// Monitor whether anyone is subscribed to the output
				image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ImageMotionNd::connectCb, this);
				// Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				img_pub  = it->advertise("/oculus2wd/image_motion_nd",  1, connect_cb, connect_cb);
			}

			// Handles (un)subscribing when clients (un)subscribe
			void connectCb()
			{
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				if (img_pub.getNumSubscribers() == 0) {
					img_sub.shutdown();
				} else if (!img_sub) {
					image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
					img_sub = it->subscribe("/camera/rgb/image_raw", 1, &ImageMotionNd::imageCb, this, hints);
				}
			}

			void imageCb(const sensor_msgs::ImageConstPtr& msg)
			{
				int x, y;
				int low_t = 25, high_t = 150, gaussian_kern = 5 , count = 0, kern_size = 3,frame_n=0,occur=0;
				float deviation = 1.41;
				msgbuf.push_back(msg);
				NODELET_DEBUG("tick %d\n", msg->header.seq);
				if(msgbuf.size() >= MSGBUF_SIZE) {
					cv::Mat msrc[MSGBUF_SIZE-1], mold, mnew, mmask, mout;
					cv_bridge::CvImagePtr cv_ptr;
					sensor_msgs::ImageConstPtr pmsg;
					bool first = true;
					for(int i=0; i<MSGBUF_SIZE; i++) {
						pmsg = msgbuf.front();
						cv_ptr = cv_bridge::toCvCopy(pmsg, sensor_msgs::image_encodings::BGR8);
						msgbuf.pop_front();
						IplImage *ipl_canny_img = cvCreateImage(cv_ptr->image.size(), IPL_DEPTH_8U, 1);
						IplImage *ipl_gray_img = cvCreateImage(cv_ptr->image.size(), IPL_DEPTH_8U, 1);
						IplImage *ipl_moving_ave = cvCreateImage(cv_ptr->image.size(), IPL_DEPTH_32F, 1);
						cv::Mat canny_img = ipl_canny_img;
						cv::Mat gray_img = ipl_gray_img;
						cv::Mat moving_ave = ipl_moving_ave;
						cv::Mat tmp, diff;

//						cv::Rect rect = cvRect (300,0,300,300);
//						cv::Mat roi_frame = cv_ptr->image(rect);
//						cv::Mat roi_gray = gray_img(rect);
//						cv::Mat roi_canny = canny_img(rect);
//						cv::Mat roi_moving = moving_ave(rect);

//						cv::rectangle(cv_ptr->image, cvPoint(300,0), cvPoint (600,300), cvScalar(255,255,0,0),1, 8, 0 );

						cv::cvtColor(cv_ptr->image, gray_img, CV_RGB2GRAY);
						GaussianBlur(gray_img, gray_img, cvSize(gaussian_kern, 0), deviation, 0);
						cv::Canny(gray_img, canny_img, low_t, high_t, kern_size);

						if(first) {
							canny_img.copyTo(diff);
							canny_img.copyTo(tmp);
							canny_img.convertTo(moving_ave, moving_ave.type(), 1, 0);
							//printf("First scale conversion done\n");
							first = false;
						} else {
							cv::accumulateWeighted(canny_img, moving_ave, 5, cv::Mat());
						}

						moving_ave.convertTo(tmp, tmp.type(), 1, 0);
						absdiff(canny_img, tmp, diff);

						std::vector< std::vector<cv::Point> > contours;
						std::vector< cv::Vec4i > hierarchy;

						cv::findContours(canny_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

						// Draw contours
						cv::Mat drawing = cv::Mat::zeros(canny_img.size(), CV_8UC3);
//						NODELET_INFO("Contours size=%d\n", contours.size());
						for( int i = 0; i< contours.size(); i++ )
						{
							cv::Scalar color = cvScalar(rand()%255, rand()%255, rand()%255);
							drawContours(cv_ptr->image, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
						}


						// Create updated CameraInfo message
						cv_bridge::CvImage out_msg;
						out_msg.header   = msg->header; // Same timestamp and tf frame as input image
						out_msg.encoding = sensor_msgs::image_encodings::RGB8;
						out_msg.image    = cv_ptr->image; // Your cv::Mat
						img_pub.publish(out_msg.toImageMsg());

						cvReleaseImage(&ipl_moving_ave);
						cvReleaseImage(&ipl_gray_img);
						cvReleaseImage(&ipl_canny_img);
					}
				}
			}

			boost::circular_buffer<sensor_msgs::ImageConstPtr> msgbuf;
			boost::mutex connect_mutex;
			boost::shared_ptr<image_transport::ImageTransport> it;
			image_transport::Subscriber img_sub;
			image_transport::Publisher img_pub;
	};

	PLUGINLIB_DECLARE_CLASS(image_motion_nd, ImageMotionNd, image_motion_nd::ImageMotionNd, nodelet::Nodelet);
}
