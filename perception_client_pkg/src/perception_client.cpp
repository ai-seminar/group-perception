#include "ros/ros.h"
#include "perception_group_msgs/AddTwoInts.h"
#include "perception_group_msgs/GetClusters.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<perception_group_msgs::AddTwoInts>("add_two_ints");
  perception_group_msgs::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

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
		}
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
