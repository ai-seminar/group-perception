#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 * TODO 
 */
void cb_(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*inputCloud,*cloud_in );

	pcl::PCDWriter writer;
	writer.write("clound_in.pcd",*cloud_in);

  ROS_INFO("Wrote a new point cloud: size = %d",cloud_in->points.size());
	ros::shutdown();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 10, cb_);
	ros::spin();
  return 0;
}
