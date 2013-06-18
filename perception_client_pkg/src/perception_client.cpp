#include "ros/ros.h"
#include "perception_group_msgs/AddTwoInts.h"
#include "perception_group_msgs/GetClusters.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

	ros::ServiceClient clusterClient = n.serviceClient<perception_group_msgs::GetClusters>("GetClusters");
  perception_group_msgs::GetClusters clusterSrv;
  clusterSrv.request.s = "foobar";

  // run until service gets shut down
  while(true)
  {
    if (clusterClient.call(clusterSrv))
    {
      // ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      ROS_INFO("Cluster Service call successful");
      ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );
      // wait a sec if list is empty. service may not be ready yet
      if((long int)clusterSrv.response.perceivedObjs.size() == 0)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_INFO_STREAM("No objects perceived yet.");
        continue;
      }
      // ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );
      for(int i=0; i < clusterSrv.response.perceivedObjs.size(); i++ ) {
        ROS_INFO("ID of perceived object is: %d", clusterSrv.response.perceivedObjs[i].c_id);
        ROS_INFO("Volume of perceived object is: %f", clusterSrv.response.perceivedObjs[i].c_volume);
        ROS_INFO("Centroid(x) of perceived object is: %f , %f , %f ",
          clusterSrv.response.perceivedObjs[i].c_centroid.x,
          clusterSrv.response.perceivedObjs[i].c_centroid.y,
          clusterSrv.response.perceivedObjs[i].c_centroid.z);
      }
      ROS_INFO_STREAM("------------------------------------------------------------");
    }
    else
    {
      ROS_ERROR("Failed to call service GetClusters");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
