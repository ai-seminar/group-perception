#include "ros/ros.h"
#include "perception_group_msgs/AddTwoInts.h"
#include "perception_group_msgs/GetClusters.h"
#include "perception_group_msgs/PerceivedObject.h"
#include "geometry_msgs/Point.h"

bool add(perception_group_msgs::AddTwoInts::Request  &req,
         perception_group_msgs::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

bool getClusters(perception_group_msgs::GetClusters::Request  &req,
         perception_group_msgs::GetClusters::Response &res)
{
	ROS_INFO("Request was ");
	ROS_INFO(req.s.c_str());
	std::vector<perception_group_msgs::PerceivedObject> perceivedObjects;
	perception_group_msgs::PerceivedObject objectOne;
	objectOne.c_id=23;
	geometry_msgs::Point centroid;
	centroid.x=23;
	centroid.y=5;
	objectOne.c_centroid = centroid;
	objectOne.c_volume = 0.23f;

	// geometry_msgs/Point c_centroid
	// float32 c_volume



	perceivedObjects.push_back(objectOne);
	res.perceivedObjs = perceivedObjects;

  // res.sum = req.a + req.b;
  // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  // ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::ServiceServer clusterService = n.advertiseService("GetClusters", getClusters);
  ROS_INFO("Ready to get clusters");
  ros::spin();

  return 0;
}
