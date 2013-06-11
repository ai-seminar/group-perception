#include "ros/ros.h"
#include "perception_group_msgs/AddTwoInts.h"
#include "perception_group_msgs/GetClusters.h"

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
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
