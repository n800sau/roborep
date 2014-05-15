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
#include <boost/thread/mutex.hpp>
#include <opencv2/video/background_segm.hpp>

namespace segment_object_nd
{

	class SegmentObjectNd : public nodelet::Nodelet
	{
		public:
			SegmentObjectNd() : update_bg_model(true) {}

		private:
			virtual void onInit()
			{
				ros::NodeHandle &nh = getNodeHandle();
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				it.reset(new image_transport::ImageTransport(nh));
				bgsubtractor.set("noiseSigma", 10);
				// Monitor whether anyone is subscribed to the output
				image_transport::SubscriberStatusCallback connect_cb = boost::bind(&SegmentObjectNd::connectCb, this);
				// Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				img_pub	 = it->advertise("/oculus2wd/segment_object_nd",  1, connect_cb, connect_cb);
			}

			// Handles (un)subscribing when clients (un)subscribe
			void connectCb()
			{
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				if (img_pub.getNumSubscribers() == 0) {
					img_sub.shutdown();
				} else if (!img_sub) {
					image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
					img_sub = it->subscribe("/camera/rgb/image_raw", 1, &SegmentObjectNd::imageCb, this, hints);
				}
			}

			void imageCb(const sensor_msgs::ImageConstPtr& msg)
			{
				cv::Mat bgmask;
				cv_bridge::CvImagePtr cv_ptr;
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

				bgsubtractor(cv_ptr->image, bgmask, update_bg_model ? -1 : 0);
				refineSegments(cv_ptr->image, bgmask, cv_ptr->image);

				// Create updated CameraInfo message
				cv_bridge::CvImage out_msg;
				out_msg.header	 = msg->header; // Same timestamp and tf frame as input image
				out_msg.encoding = msg->encoding;
				out_msg.image	 = cv_ptr->image; // Your cv::Mat
				img_pub.publish(out_msg.toImageMsg());
			}

			void refineSegments(const cv::Mat& img, cv::Mat& mask, cv::Mat& dst)
			{
				int niters = 3;

				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;

				cv::Mat temp;

				cv::dilate(mask, temp, cv::Mat(), cv::Point(-1,-1), niters);
				cv::erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters*2);
				cv::dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);

				cv::findContours( temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

				dst = cv::Mat::zeros(img.size(), CV_8UC3);

				if( contours.size() == 0 )
					return;

				// iterate through all the top-level contours,
				// draw each connected component with its own random color
				int idx = 0, largestComp = 0;
				double maxArea = 0;

				for( ; idx >= 0; idx = hierarchy[idx][0] )
				{
					const std::vector<cv::Point>& c = contours[idx];
					double area = fabs(contourArea(cv::Mat(c)));
					if( area > maxArea )
					{
						maxArea = area;
						largestComp = idx;
					}
				}
				cv::Scalar color( 0, 0, 255 );
				cv::drawContours( dst, contours, largestComp, color, CV_FILLED, 8, hierarchy );
			}

			boost::mutex connect_mutex;
			boost::shared_ptr<image_transport::ImageTransport> it;
			image_transport::Subscriber img_sub;
			image_transport::Publisher img_pub;
			cv::BackgroundSubtractorMOG bgsubtractor;
			bool update_bg_model;
			cv::Mat bgmask;
	};

	PLUGINLIB_DECLARE_CLASS(segment_object_nd, SegmentObjectNd, segment_object_nd::SegmentObjectNd, nodelet::Nodelet);
}
