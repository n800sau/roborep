#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{

  cloudrgbptr PC (new cloudrgb());
  cloudrgbptr PC_filtered (new cloudrgb());
  pcl::fromROSMsg(*cloud, *PC); //Now you can process this PC using the pcl functions 
  sensor_msgs::PointCloud2 cloud_filtered;

//	int cloudsize = (PC->width) * (PC->height);
//	for (int i=0; i< cloudsize; i++){
//		rgbpoint pnt = PC->points[i];
//		if(pnt.x == pnt.x) {
//			std::cout << "PNT(x,y,z) = " << pnt.x << pnt.y << pnt.z << pnt.x << std::endl;
//		}
//	}

	std::cerr << "PointCloud before filtering: " << cloud->width << "x" << cloud->height << "\n";

// Perform the actual filtering
   pcl::VoxelGrid<pcl::PointXYZRGB> sor ;
   sor.setInputCloud (PC);
   sor.setLeafSize (0.01, 0.01, 0.01);
   sor.filter (*PC_filtered);

// pcl::PCDWriter writer;
//  writer.write ("table_scene_downsampled.pcd", *PC_filtered, 
//         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);



  //Convert the pcl cloud back to rosmsg
  pcl::toROSMsg(*PC_filtered, cloud_filtered);
  //Set the header of the cloud
  cloud_filtered.header.frame_id = cloud->header.frame_id;
  // Publish the data
  //You may have to set the header frame id of the cloud_filtered also
  pub.publish (cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered.width << "x" << cloud_filtered.height << " data points (" << pcl::getFieldsList (cloud_filtered) << ").\n";


}
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_voxel");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/oculus2wd/processed_points", 1);
    ros::spin();
}

