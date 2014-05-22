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

namespace image_motion1_nd
{

	class ImageMotion1Nd : public nodelet::Nodelet
	{
		public:
			ImageMotion1Nd() : step(2), number_of_sequence(0) {}

		private:
			virtual void onInit()
			{
				ros::NodeHandle &nh = getNodeHandle();
				ros::NodeHandle &private_nh = getPrivateNodeHandle();
				it.reset(new image_transport::ImageTransport(nh));
				// Monitor whether anyone is subscribed to the output
				image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ImageMotion1Nd::connectCb, this);
				// Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				res_pub = it->advertise("/oculus2wd/image_motion1_nd", 1, connect_cb, connect_cb);
				cropped_pub = it->advertise("/oculus2wd/cropped_image_motion1_nd", 1, connect_cb, connect_cb);
				// Erode kernel
				kernel_ero = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
			}

			// Handles (un)subscribing when clients (un)subscribe
			void connectCb()
			{
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				if (res_pub.getNumSubscribers() == 0) {
					img_sub.shutdown();
				} else if (!img_sub) {
					image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
					img_sub = it->subscribe("/camera/rgb/image_raw", 1, &ImageMotion1Nd::imageCb, this, hints);
				}
			}

			// Check if there is motion in the result matrix
			// count the number of changes and return.
			int detectMotion(const cv::Mat & motion, cv::Mat & result, cv::Mat & result_cropped,
					int x_start, int x_stop, int y_start, int y_stop,
					int max_deviation, cv::Scalar & color)
			{
				int number_of_changes = 0;
				// calculate the standard deviation
				cv::Scalar mean, stddev;
				cv::meanStdDev(motion, mean, stddev);
				// if noto to much changes then the motion is real (neglect agressive snow, temporary sunlight)
				if(stddev[0] < max_deviation)
				{
					int min_x = motion.cols, max_x = 0;
					int min_y = motion.rows, max_y = 0;
					// loop over image and detect changes
					for(int j = y_start; j < y_stop; j+=2){ // height
						for(int i = x_start; i < x_stop; i+=2){ // width
							// check if at pixel (j,i) intensity is equal to 255
							// this means that the pixel is different in the sequence
							// of images (prev_frame, current_frame, next_frame)
							if(static_cast<int>(motion.at<uchar>(j,i)) == 255)
							{
								number_of_changes++;
								if(min_x>i) min_x = i;
								if(max_x<i) max_x = i;
										if(min_y>j) min_y = j;
								if(max_y<j) max_y = j;
							}
						}
					}
					if (number_of_changes) {
						//check if not out of bounds
						if(min_x-10 > 0) min_x -= 10;
						if(min_y-10 > 0) min_y -= 10;
						if(max_x+10 < result.cols-1) max_x += 10;
						if(max_y+10 < result.rows-1) max_y += 10;
						// draw rectangle round the changed pixel
						cv::Point x(min_x,min_y);
						cv::Point y(max_x,max_y);
						cv::Rect rect(x,y);
						NODELET_INFO("Rect:%d,%d-%d,%d\n", min_x, min_y, max_x, max_y);
						cv::Mat cropped = result(rect);
						cropped.copyTo(result_cropped);
						cv::rectangle(result,rect,color,1);
					}
				}
				return number_of_changes;
			}

			void imageCb(const sensor_msgs::ImageConstPtr& msg)
			{
//				NODELET_INFO("step=%d", step);
				cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				// d1 and d2 for calculating the differences
				// result, the result of and operation, calculated on d1 and d2
				// number_of_changes, the amount of changes in the result matrix.
				// color, the color for drawing the rectangle when something has changed.
				cv::Mat d1, d2, motion, result, result_cropped;
				int number_of_changes;
				cv::Scalar color(255, 0, 0);
				// Detect motion in window
				int x_start = 1, x_stop = cv_ptr->image.cols-1;
				int y_start = 1, y_stop = cv_ptr->image.rows-1;
				// If more than 'there_is_motion' pixels are changed, we say there is motion
				int there_is_motion = 1;
				// Maximum deviation of the image, the higher the value, the more motion is allowed
				int max_deviation = 1000;
				switch(step) {
					case 2:
						prev_frame = cv_ptr->image;
						cv::cvtColor(prev_frame, prev_frame, CV_RGB2GRAY);
						step = 1;
						break;
					case 1:
						current_frame = cv_ptr->image;
						cv::cvtColor(current_frame, current_frame, CV_RGB2GRAY);
						step = 0;
						break;
					default:
						next_frame = cv_ptr->image;
						cv::cvtColor(next_frame, next_frame, CV_RGB2GRAY);
						// Calc differences between the images and do AND-operation
						// threshold image, low differences are ignored (ex. contrast change due to sunlight)
						next_frame.copyTo(result);
						cv::cvtColor(result, result, CV_GRAY2BGR);
						cv::absdiff(prev_frame, next_frame, d1);
						cv::absdiff(next_frame, current_frame, d2);
						cv::bitwise_and(d1, d2, motion);
						cv::threshold(motion, motion, 35, 255, CV_THRESH_BINARY);
						cv::erode(motion, motion, kernel_ero);
						number_of_changes = detectMotion(motion, result, result_cropped,  x_start, x_stop, y_start, y_stop, max_deviation, color);
						// If a lot of changes happened, we assume something changed.
						if(number_of_changes >= there_is_motion)
						{
							if(number_of_sequence>0) {
		//						NODELET_INFO("Number of changes=%d", number_of_changes);
								cv_bridge::CvImage result_msg;
								result_msg.header = msg->header; // Same timestamp and tf frame as input image
								result_msg.encoding = sensor_msgs::image_encodings::BGR8;
								result_msg.image = result;
								res_pub.publish(result_msg.toImageMsg());
								cv_bridge::CvImage cropped_msg;
								cropped_msg.header = msg->header; // Same timestamp and tf frame as input image
								cropped_msg.encoding = sensor_msgs::image_encodings::BGR8;
								cropped_msg.image = result_cropped;
								cropped_pub.publish(cropped_msg.toImageMsg());
							}
							number_of_sequence++;
						} else {
							number_of_sequence = 0;
						}
						current_frame.copyTo(prev_frame);
						next_frame.copyTo(current_frame);
						break;
				}
			}

			boost::mutex connect_mutex;
			boost::shared_ptr<image_transport::ImageTransport> it;
			image_transport::Subscriber img_sub;
			image_transport::Publisher res_pub;
			image_transport::Publisher cropped_pub;
			int step, number_of_sequence;
			cv::Mat prev_frame, current_frame, next_frame;
			cv::Mat kernel_ero;
	};

	PLUGINLIB_DECLARE_CLASS(image_motion1_nd, ImageMotion1Nd, image_motion1_nd::ImageMotion1Nd, nodelet::Nodelet);
}
