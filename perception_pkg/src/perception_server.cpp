/**
 * perception_server.cpp
 *
 * Process a complete point cloud from ROS
 * and calculate the volume and centroid of the objects.
 */

#include <algorithm>
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

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// Comparator function for PerceivedObject's. PerceivedObjects will be compared by their volume
bool ReceivedObjectGreaterThan(const perception_group_msgs::PerceivedObject& p1, const perception_group_msgs::PerceivedObject& p2){
	return p1.c_volume > p2.c_volume;
}

/*
 * Base class for the perception server
 *
 * This class advertises the GetClusters service when it is constructed.
 * For every service request, the server will subscribe to the /camera/depth_registered/points topic
 * and take exactly one point cloud.
 * Afterwards, it will process it, e.g. calculate the volume and centroid of every PerceivedObject, and return the results
 * to the service requester.
 */
class PerceptionServer
{
	private:
		// Mutex for buffer locking
		boost::signals2::mutex mutex;
		// Buffer for the last perceived objects
		std::vector<perception_group_msgs::PerceivedObject> perceivedObjects;
		// Indicates that a subscription request is in process.
		bool processing;
		// ID counter for the perceived objects
		int objectID;

		ros::NodeHandle n;
		ros::ServiceServer clusterService;
		
	public:
		PerceptionServer(ros::NodeHandle& n_);
		void process_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
		void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
		bool getClusters(perception_group_msgs::GetClusters::Request  &req,
						 perception_group_msgs::GetClusters::Response &res);
};

	/*
	 * Constructor for the PerceptionServer class.
	 * Advertises the GetClusters service
	 */
  PerceptionServer::PerceptionServer(ros::NodeHandle& n_) : n(n_)
  {
    // Advertise Service
    clusterService = n.advertiseService("GetClusters", 
      &PerceptionServer::getClusters, this);
    objectID = 0;
  }

	/*
	 * Process a single point cloud.
	 * This will include
	 *   - Centroid calculation
	 *   - Volume calculation
	 *
	 * The result is a list of PerceivedObject's, which will be put into the buffer perceivedObjects.
	 */
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

		// Use cluster extraction to get rid of the outliers of the segmented table
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeTable (new pcl::search::KdTree<pcl::PointXYZRGB>);
		treeTable->setInputCloud (cloud_plane);  
		std::vector<pcl::PointIndices> table_cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecTable;
		ecTable.setClusterTolerance (0.02); // 2cm
		ecTable.setMinClusterSize (10000);
		ecTable.setMaxClusterSize (200000);
		ecTable.setSearchMethod (treeTable);
		ecTable.setInputCloud (cloud_plane);
		ecTable.extract (table_cluster_indices);

		// Extract the biggest cluster (e.g. the table) in the plane cloud
		std::vector<pcl::PointIndices>::const_iterator it = table_cluster_indices.begin ();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			plane_cluster->points.push_back (cloud_plane->points[*pit]); //*
		plane_cluster->width = plane_cluster->points.size ();
		plane_cluster->height = 1;
		plane_cluster->is_dense = true;

		std::cout << "Table point cloud " << plane_cluster->points.size () << " data points." << std::endl;
    
		// Remove the plane from the rest of the point cloud
    extract.setNegative(true);
    extract.filter(*cloud_clusters);

    // Use ExtractPolygonalPrism to get all the point clouds above the plane in a given range
    double z_min = 0., z_max = 0.50; // we want the points above the plane, no farther than 50 cm from the surface
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);

    hull.setDimension (2); 
    hull.setInputCloud (plane_cluster);
    hull.reconstruct (*hull_points);
      
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
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cluster_cloud_filtered);
    ec.extract (cluster_indices);

    // temporary list of perceived objects
    std::vector<perception_group_msgs::PerceivedObject> tmpPerceivedObjects;

		// Iterate over the extracted clusters and write them as a PerceivedObjects to the result list
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
		// Sort by volume
		std::sort(tmpPerceivedObjects.begin(), tmpPerceivedObjects.end(), ReceivedObjectGreaterThan);
   
		// Lock the buffer access to assign the recently perceived objects
    mutex.lock();
    perceivedObjects=tmpPerceivedObjects;
    mutex.unlock();
  }

	/*
	 * Receive callback for the /camera/depth_registered/points subscription
	 */
  void PerceptionServer::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
  {

		// process only one cloud
		if(processing){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*inputCloud,*cloud_in);

      process_cloud(cloud_in);
      processing = false;

      ROS_INFO("Received a new point cloud: size = %d",cloud_in->points.size());
		}
  }

	/*
	 * Implementation of the GetClusters Service.
	 *
	 * This method will subscribe to the /camera/depth_registered/points topic, 
	 * wait for the processing of a single point cloud, and return the result from
	 * the calulations as a list of PerceivedObjects.
	 */
  bool PerceptionServer::getClusters(perception_group_msgs::GetClusters::Request &req,
           perception_group_msgs::GetClusters::Response &res)
  {
		ros::Subscriber sub;
    processing = true;

    // Subscribe to the depth information topic
    sub = n.subscribe("/camera/depth_registered/points", 1, 
      &PerceptionServer::receive_cloud, this);
    
    ROS_INFO("Waiting for processed cloud");
		ros::Rate r(20); // 20 hz
    while(processing){
			ros::spinOnce();
			r.sleep();
		}
    ROS_INFO("Cloud processed. Lock buffer and return the results");

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
