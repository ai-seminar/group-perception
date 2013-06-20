/**
 * perception_server.cpp
 *
 * Process a complete point cloud from ROS
 * and calculate the volume and centroid of the objects.
 */
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include "perception_group_msgs/PerceivedObject.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread/thread.hpp> 
#include "geometry_msgs/Point.h"
#include "perception_group_msgs/GetClusters.h"

class PerceptionServer
{
private:
  boost::signals2::mutex mutex;
  std::vector<perception_group_msgs::PerceivedObject> perceivedObjects;
  bool processing;
  ros::NodeHandle n;
  ros::ServiceServer clusterService;
  ros::Subscriber sub;
  int objectID;
  
public:
  PerceptionServer(ros::NodeHandle& n_);
  void process_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
  void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  bool getClusters(perception_group_msgs::GetClusters::Request  &req,
           perception_group_msgs::GetClusters::Response &res);
};

  PerceptionServer::PerceptionServer(ros::NodeHandle& n_) : n(n_)
  {
    // Advertise Service
    clusterService = n.advertiseService("GetClusters", 
      &PerceptionServer::getClusters, this);
    objectID = 0;
  }

  void PerceptionServer::process_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
  {
    //point cloud objects
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    //removing nans from point clouds
    std::vector<int> nans;
    pcl::removeNaNFromPointCloud(*cloud_in,*cloud_nanles,nans);

    //filtering cloud on z axis
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_nanles);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.5);
    pass.filter(*cloud_filtered);

    //voxelizing cloud
    pcl::VoxelGrid <pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.01f,0.01f,0.01f);
    vg.filter(*cloud_downsampled);

    //fitting a plane to the filtered cloud
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers,*coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      exit(0);
    }

    //splitting the cloud in two: plane + other
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.filter(*cloud_plane);

    // Remove the plane from the rest of the point cloud
    extract.setNegative(true);
    extract.filter(*cloud_clusters);

    // Create a ConvexHull for the table plane
    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    // Use ExtractPolygonalPrism to get all the point clouds above the plane in a given range
    
    double z_min = 0., z_max = 0.50; // we want the points above the plane, no farther than 50 cm from the surface
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);

    // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
    hull.setInputCloud (cloud_plane);
    hull.reconstruct (*hull_points);
    if (hull.getDimension () == 2)
    {
      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
      prism.setInputCloud (cloud_clusters);
      prism.setInputPlanarHull (hull_points);
      prism.setHeightLimits (z_min, z_max);
      prism.segment (*object_indices);

      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZRGB> extractObjects;
      // Extract the inliers of the prism
      extract.setInputCloud (cloud_clusters);
      extract.setIndices (object_indices);
      extract.setNegative (false);
      extract.filter (*object_clusters);
    }
    else
     ROS_ERROR ("The input cloud does not represent a planar surface.\n");


    // cluster extraction

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg_clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg_clusters.setInputCloud (object_clusters);
    vg_clusters.setLeafSize (0.01f, 0.01f, 0.01f);
    vg_clusters.filter (*cluster_cloud_filtered);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cluster_cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.03); // 2cm
    ec.setMinClusterSize (100);
    // ec.setMaxClusterSize (25000);
    ec.setMaxClusterSize (4000); // avoid the stuff in the back
    ec.setSearchMethod (tree);
    ec.setInputCloud (cluster_cloud_filtered);
    ec.extract (cluster_indices);

    // temporary list of perceived objects
    std::vector<perception_group_msgs::PerceivedObject> tmpPerceivedObjects;

    std::cout << "Loop start: " << boost::posix_time::microsec_clock::universal_time() << std::endl;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back (cluster_cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Calculate the volume of each cluster
      // Create a convex hull around the cluster and calculate the total volume
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::ConvexHull<pcl::PointXYZRGB> hull;
      hull.setInputCloud (cloud_cluster);
      hull.setDimension(3);
      hull.setComputeAreaVolume(true); // This creates alot of output, but it's necessary for getTotalVolume() ....
      hull.reconstruct (*hull_points);

      // Centroid calulcation
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*hull_points, centroid);  
      std::cout << "The centroid vector is: " << centroid[0] << ","<<centroid[1]<<"," << centroid[2] <<","<< centroid[3] <<std::endl;

      // Add the detected cluster to the list of perceived objects
      perception_group_msgs::PerceivedObject percObj;
      percObj.c_id= objectID;
      objectID++;
      geometry_msgs::Point ptCentroid;
      ptCentroid.x=centroid[0];
      ptCentroid.y=centroid[1];
      ptCentroid.z=centroid[2];
      percObj.c_centroid = ptCentroid;
      percObj.c_volume = hull.getTotalVolume();

      tmpPerceivedObjects.push_back(percObj);

      j++;
    }
    
    mutex.lock();
    perceivedObjects=tmpPerceivedObjects;
    mutex.unlock();
  }

  void PerceptionServer::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
  {
    ROS_INFO("CALLBACK");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*inputCloud,*cloud_in);

      process_cloud(cloud_in);
      processing = false;

      ROS_INFO("Wrote a new point cloud: size = %d",cloud_in->points.size());
  }

  bool PerceptionServer::getClusters(perception_group_msgs::GetClusters::Request &req,
           perception_group_msgs::GetClusters::Response &res)
  {
    ROS_INFO("Request was ");
    ROS_INFO(req.s.c_str());
    processing = true;

    // Subscribe to the depth information topic
    sub = n.subscribe("/camera/depth_registered/points", 2, 
      &PerceptionServer::receive_cloud, this);
    
    ROS_INFO("Waiting for processed cloud");
    while(processing)
      boost::this_thread::sleep(boost::posix_time::milliseconds(10));

    mutex.lock();
    res.perceivedObjs = perceivedObjects;
    mutex.unlock();

    return true;
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  PerceptionServer ps(nh);

  ROS_INFO("Ready to get clusters");
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
