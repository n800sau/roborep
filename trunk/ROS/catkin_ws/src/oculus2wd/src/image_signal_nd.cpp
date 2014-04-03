#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp> //This is where actual SURF and SIFT algorithm is located
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

#define MSGBUF_SIZE 2

namespace image_signal_nd
{

	class ImageSignalNd : public nodelet::Nodelet
	{
		public:
			ImageSignalNd() : msgbuf(MSGBUF_SIZE) {}

		private:
			virtual void onInit()
			{
				ros::NodeHandle &nh = getNodeHandle();
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				it.reset(new image_transport::ImageTransport(nh));
				// Monitor whether anyone is subscribed to the output
				typedef image_transport::SubscriberStatusCallback ConnectCB;
				ConnectCB connect_cb = boost::bind(&ImageSignalNd::connectCb, this);
				// Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				img_pub  = it->advertise("/oculus2wd/signal_image_nd",  1, connect_cb, connect_cb);
			}

			// Handles (un)subscribing when clients (un)subscribe
			void connectCb()
			{
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				if (img_pub.getNumSubscribers() == 0) {
					img_pub.shutdown();
				} else if (!img_sub) {
					image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
					img_sub = it->subscribe("/camera/rgb/image_raw", 1, &ImageSignalNd::imageCb, this, hints);
				}
			}

			void imageCb(const sensor_msgs::ImageConstPtr& msg)
			{
				int x, y;
				msgbuf.push_back(msg);
//				printf("tick %d\n", msg->header.seq);
				NODELET_DEBUG("tick %d\n", msg->header.seq);
				if(msgbuf.size() >= MSGBUF_SIZE) {
					cv::Mat msrc[MSGBUF_SIZE-1], mold, mnew, mmask, mout;
					cv_bridge::CvImageConstPtr cv_ptr;
					sensor_msgs::ImageConstPtr pmsg;
					for(int i=0; i<MSGBUF_SIZE; i++) {
						pmsg = msgbuf.front();
						cv_ptr = cv_bridge::toCvShare(pmsg, sensor_msgs::image_encodings::BGR8);
						cv::cvtColor(cv_ptr->image, mnew, CV_BGR2GRAY);
						msgbuf.pop_front();
						if(i>0) {
							mmask = mnew - mold;
							cv::threshold(mmask, mmask, 100, 255, cv::THRESH_BINARY);
    
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
								mout = cv_ptr->image;
								mout.setTo(cv::Scalar(0, 0, 0));
    
								cv::rectangle(mout, pmin, pmax, cv::Scalar(255, 0, 0), 1, 8, 0);
								cv_bridge::CvImage out_msg;
								out_msg.header   = msg->header; // Same timestamp and tf frame as input image
								out_msg.encoding = pmsg->encoding;
								out_msg.image    = mout; // Your cv::Mat
								img_pub.publish(out_msg.toImageMsg());
							}
						}
						mold = mnew.clone();
					}
				}
			}

		boost::circular_buffer<sensor_msgs::ImageConstPtr> msgbuf;
		boost::mutex connect_mutex;
		boost::shared_ptr<image_transport::ImageTransport> it;
		image_transport::Subscriber img_sub;
		image_transport::Publisher img_pub;
	};

	PLUGINLIB_DECLARE_CLASS(image_signal_nd, ImageSignalNd, image_signal_nd::ImageSignalNd, nodelet::Nodelet);
}
