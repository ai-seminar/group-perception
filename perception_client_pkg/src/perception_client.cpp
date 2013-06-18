#include "ros/ros.h"
#include "perception_group_msgs/AddTwoInts.h"
#include "perception_group_msgs/GetClusters.h"
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;

	ros::ServiceClient clusterClient = n.serviceClient<perception_group_msgs::GetClusters>("GetClusters");
  perception_group_msgs::GetClusters clusterSrv;
  clusterSrv.request.s = "foobar";

  if (clusterClient.call(clusterSrv))
  {
    // ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		ROS_INFO("Cluster Service call successful");
    ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );
    // ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );
		for(int i=0; i < clusterSrv.response.perceivedObjs.size(); i++ ) {
			ROS_INFO("ID of perceived object is: %d" ,clusterSrv.response.perceivedObjs[i].c_id);
			ROS_INFO("Volume of perceived object is: %f" ,clusterSrv.response.perceivedObjs[i].c_volume);
			ROS_INFO("Centroid(x) of perceived object is: %f , %f , %f " ,clusterSrv.response.perceivedObjs[i].c_centroid.x,clusterSrv.response.perceivedObjs[i].c_centroid.y,clusterSrv.response.perceivedObjs[i].c_centroid.z);

			// geometry_msgs::Point centroid;
		}
  }
  else
  {
    ROS_ERROR("Failed to call service GetClusters");
    return 1;
  }

  return 0;
}
