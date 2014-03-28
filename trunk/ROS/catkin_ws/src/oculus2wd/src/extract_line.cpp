// to get horizontal distance line using xtion

#include <sstream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <oculus2wd/extract_lineConfig.h>
//#include <vector>

using namespace std;

ros::Publisher pub;
image_transport::Publisher img_pub;

typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;

int field_height;
int vertical_level;

int image_width;
int image_height;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{

	cloudrgbptr PC (new cloudrgb());
	cloudrgbptr PC_filtered (new cloudrgb());
	pcl::fromROSMsg(*cloud, *PC); //Now you can process this PC using the pcl functions 
	sensor_msgs::PointCloud2 cloud_filtered;

	pcl::PointXYZRGB minPt, maxPt;
	pcl::PointXYZ minLimit(-3., -0.5, 0.2), maxLimit(3., 1., 8.);

	pcl::getMinMax3D (*PC, minPt, maxPt);
//  std::cout << "x: " << minPt.x << " - " << maxPt.x << std::endl;
//  std::cout << "y: " << minPt.y << " - " << maxPt.y << std::endl;
//  std::cout << "z: " << minPt.z << " - " << maxPt.z << std::endl;

	// choose middle of y
	float middle_y = minLimit.y + (maxLimit.y - minLimit.y) * vertical_level / 100.;

  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
	for(size_t i = 0; i<PC->points.size(); ++i)
	{
		if(PC->points[i].y < middle_y + field_height / 2. && PC->points[i].y > middle_y - field_height / 2.) {
			PC_filtered->points.push_back(PC->points[i]);
		}
	}

//	std::cerr << "PointCloud before filtering: " << cloud->width << "x" << cloud->height << "\n";

//	cv::Mat image(160, 120, CV_8UC3, Scalar(0,0,0));

	IplImage* img = cvCreateImage(cvSize(image_width, image_height), 8,3);
//	std::cerr << "Image size: " << image_width << 'x' << image_height << "\n";
	cvSet(img, cv::Scalar(200, 200, 200));

	float mt_per_px = (maxLimit.z - minLimit.z) / image_height;
	float x_per_px = (maxLimit.x - minLimit.x) / image_width;

	const int n_clrs = 5;
	CvScalar colors[n_clrs] = {
		{{0, 128, 0}},
		{{0, 64, 64}},
		{{128, 0, 0}},
		{{64, 0, 64}},
		{{0, 0, 128}}
	};

	float step = (maxLimit.z - minLimit.z) / n_clrs;

//	std::vector<pcl::PointXYZRGB> vector_pnts (PC_filtered->points, PC_filtered->points + PC_filtered->points.size());

//	std::cerr << minLimit.x << ".." << maxLimit.x << "\n";

	CvPoint p;
	for(size_t i = 0; i<PC_filtered->points.size(); ++i)
	{
		int cindex = round((PC_filtered->points[i].z - minLimit.z) / step);
		if(cindex >= n_clrs) {
			cindex = n_clrs - 1;
		}
		if(cindex < 0) {
			cindex = 0;
		}
		int iy = int((PC_filtered->points[i].z - minLimit.z) / mt_per_px);
		int ix = int((PC_filtered->points[i].x - minLimit.x) / x_per_px);
		p.x = ix;
		p.y = iy;
		cvCircle(img, p, 1, colors[cindex]);
	}

//	stringstream fname;
//	fname << "line" << cloud->header.seq << ".png";
//	cvSaveImage( fname.str().c_str(), img );
	cvSaveImage( "/var/www/pcl/test.png", img );

	cv_bridge::CvImage cvi;
	cvi.header.stamp = ros::Time();
	cvi.header.frame_id = "camera";
	cvi.encoding = "bgr8";
	cvi.image = img;

	// Publish the image.
	sensor_msgs::Image::Ptr out_img = cvi.toImageMsg();
	img_pub.publish(out_img);

	cvReleaseImage(&img);

	//Convert the pcl cloud back to rosmsg
	pcl::toROSMsg(*PC_filtered, cloud_filtered);
	//Set the header of the cloud
	cloud_filtered.header.frame_id = cloud->header.frame_id;
	// Publish the data
	//You may have to set the header frame id of the cloud_filtered also
	pub.publish (cloud_filtered);
//	std::cerr << "PointCloud after filtering: " << cloud_filtered.width << "x" << cloud_filtered.height << " data points (" << pcl::getFieldsList (cloud_filtered) << ").\n";

}

void reconfigure_callback(oculus2wd::extract_lineConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %d %dx%d",
		config.vertical_level, config.field_height,
		config.image_width, config.image_height);
	vertical_level = config.vertical_level;
	field_height = config.field_height;
	image_width = config.image_width;
	image_height = config.image_height;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "extract_line");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 5, cloud_cb);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/oculus2wd/hline", 1);
	image_transport::ImageTransport it(nh);
	img_pub = it.advertise("/extract_line/image", 1);
	dynamic_reconfigure::Server<oculus2wd::extract_lineConfig> server;
	dynamic_reconfigure::Server<oculus2wd::extract_lineConfig>::CallbackType f;
	f = boost::bind(&reconfigure_callback, _1, _2);
	server.setCallback(f);
	ros::spin();
}
