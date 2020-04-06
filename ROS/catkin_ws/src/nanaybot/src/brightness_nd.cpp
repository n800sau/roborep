#include <boost/version.hpp> 
#if ((BOOST_VERSION / 100) % 1000) >= 53 
#include <boost/thread/lock_guard.hpp> 
#endif 

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

#define MSGBUF_SIZE 2

namespace brightness_nd
{

	class BrightnessNd : public nodelet::Nodelet
	{
		public:
			BrightnessNd() {}

		private:
			virtual void onInit()
			{
				ros::NodeHandle &nh = getNodeHandle();
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				it.reset(new image_transport::ImageTransport(nh));
				// Monitor whether anyone is subscribed to the output
				ros::SubscriberStatusCallback connect_cb = boost::bind(&BrightnessNd::connectCb, this);
				// Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				pub = nh.advertise<std_msgs::Float32> ("/brightness", 1, connect_cb, connect_cb);
			}

			// Handles (un)subscribing when clients (un)subscribe
			void connectCb()
			{
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				if (pub.getNumSubscribers() == 0) {
					img_sub.shutdown();
				} else if (!img_sub) {
					image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
					img_sub = it->subscribe("/camera/rgb/image_raw", 1, &BrightnessNd::imageCb, this, hints);
				}
			}

			void imageCb(const sensor_msgs::ImageConstPtr& msg)
			{
				int x, y;
				cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
				double val = *cv::mean(cv_ptr->image).val / 256;

				std_msgs::Float32 out_msg;
				out_msg.data  = val;
				pub.publish(out_msg);
			}

			boost::mutex connect_mutex;
			boost::shared_ptr<image_transport::ImageTransport> it;
			image_transport::Subscriber img_sub;
			ros::Publisher pub;
	};

	PLUGINLIB_DECLARE_CLASS(brightness_nd, BrightnessNd, brightness_nd::BrightnessNd, nodelet::Nodelet);
}
