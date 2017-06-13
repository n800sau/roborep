#include <boost/version.hpp> 
#if ((BOOST_VERSION / 100) % 1000) >= 53 
#include <boost/thread/lock_guard.hpp> 
#endif 

#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/unordered_map.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

using namespace raspicam;

namespace raspicam_nd
{

	class RaspiCamNd : public nodelet::Nodelet
	{
		public:
			RaspiCamNd():camera_cv() {
				color_mode_map["mono8"] = CV_8UC1;
				color_mode_map["rgb8"] = CV_8UC3;
				camera_cv.set(CV_CAP_PROP_FORMAT, CV_8UC1);
			}

		private:
			virtual void onInit()
			{
				NODELET_DEBUG("RaspiCamNd onInit start");
				ros::NodeHandle &nh = getNodeHandle();
				ros::NodeHandle &private_nh = getPrivateNodeHandle();

				it.reset(new image_transport::ImageTransport(nh));

//				camera_name = nh.getNamespace();
//				NODELET_DEBUG_STREAM("Camera name: " << camera_name);
//				cinfo.reset(new camera_info_manager::CameraInfoManager(nh, camera_name));
				cinfo.reset(new camera_info_manager::CameraInfoManager(nh, "pi_camera"));

				std::string url;
				private_nh.param<std::string>("camera_info_url", url, "file://" + ros::package::getPath("raspicam_nd") + "/pi_camera.yaml");
//				private_nh.param<std::string>("camera_info_url", url, "");
				NODELET_DEBUG_STREAM("YAML url: " << url);
				cinfo->loadCameraInfo(url);

				// Monitor whether anyone is subscribed to the output
				ros::SubscriberStatusCallback connect_cb = boost::bind(&RaspiCamNd::connectCb, this);
				// Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				image_transport::SubscriberStatusCallback image_cb = boost::bind(&RaspiCamNd::connectCb, this);
				pub = it->advertiseCamera("image_raw", 1, image_cb, image_cb);
			}

			// Handles (un)subscribing when clients (un)subscribe
			void connectCb()
			{
				boost::lock_guard<boost::mutex> lock(connect_mutex);
				if (pub.getNumSubscribers() == 0) {
					if(camera_cv.isOpened()) {
						timer_.stop();
						NODELET_DEBUG("RaspiCamNd Disconnect callback!");
						camera_cv.release();
					}
				} else {
					if(!camera_cv.isOpened()) {
						NODELET_DEBUG("RaspiCamNd Connect callback!");
						ros::NodeHandle &nh = getNodeHandle();
						ros::NodeHandle &private_nh = getPrivateNodeHandle();
						int fps, width, height;
						private_nh.param("fps", fps, 10);
						private_nh.param<std::string>("color_mode", color_mode, "rgb8");
						private_nh.param("width", width, 320);
						private_nh.param("height", height, 240);

						camera_cv.set(CV_CAP_PROP_FORMAT, color_mode_map[color_mode]);
						camera_cv.set(CV_CAP_PROP_FPS, fps);
						camera_cv.set(CV_CAP_PROP_FRAME_WIDTH, width);
						camera_cv.set(CV_CAP_PROP_FRAME_HEIGHT, height);
						camera_cv.set(CV_CAP_PROP_EXPOSURE, -1);
						camera_cv.set(CV_CAP_PROP_GAIN, 0);
						NODELET_DEBUG_STREAM("fps set to " << fps << ", color_mode: " << color_mode << ", size: " << width << "x" << height);
						if(!camera_cv.open())
							ROS_ERROR("Error opening camera");
						sleep(3);
						camera_cv.grab();

						timer_ = nh.createTimer(ros::Duration(0.5 / fps), boost::bind(&RaspiCamNd::timerCb, this, _1));
					}
				}
			}

			void timerCb(const ros::TimerEvent& event)
			{
				cv::Mat cv_img;
				camera_cv.grab();
				camera_cv.retrieve(cv_img);
				std_msgs::Header header();
				cv_bridge::CvImage imgmsg;
				sensor_msgs::CameraInfo ci = cinfo->getCameraInfo();
				imgmsg.header.frame_id = camera_name + "_optical_frame";
				ci.header.frame_id = imgmsg.header.frame_id;
				imgmsg.encoding = color_mode;
				imgmsg.image = cv_img;
				pub.publish(*imgmsg.toImageMsg(), ci, ros::Time::now());

			}

			boost::unordered_map<std::string, int> color_mode_map;
			RaspiCam_Cv camera_cv;

			boost::mutex connect_mutex;
			boost::shared_ptr<image_transport::ImageTransport> it;
			boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;
			image_transport::CameraPublisher pub;
			ros::Timer timer_;
			std::string camera_name;
			std::string color_mode;
	};

	PLUGINLIB_DECLARE_CLASS(raspicam_nd, RaspiCamNd, raspicam_nd::RaspiCamNd, nodelet::Nodelet);
}
