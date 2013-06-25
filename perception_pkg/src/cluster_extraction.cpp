/**
 * cluster_extraction.cpp
 *
 * You can use this programm to separate point clusters from each other with euclidean clustering.
 * Normally, you will call this on the resulting pcd file from objects_on_table.cpp
 * to work on the objects above the table.
 */

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

int 
main (int argc, char** argv)
{
	if(argc<2)
	{
	        std::cerr<<"Please specify a pcd file to use"<<std::endl;
	        std::cerr<<"Run program like this: cluster_extraction [filename.pcd]"<<std::endl;
		exit(0);
	}

  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.02);

  // int i=0, nr_points = (int) cloud_filtered->points.size ();
  // while (cloud_filtered->points.size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud (cloud_filtered);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0)
  //   {
  //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }

  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  //   extract.setInputCloud (cloud_filtered);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);

  //   // Get the points associated with the planar surface
  //   extract.filter (*cloud_plane);
  //   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  //   // Remove the planar inliers, extract the rest
  //   extract.setNegative (true);
  //   extract.filter (*cloud_f);
  //   *cloud_filtered = *cloud_f;
  // }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.03); // 2cm
  ec.setMinClusterSize (100);
  // ec.setMaxClusterSize (25000);
  ec.setMaxClusterSize (4000); // avoid the stuff in the back
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster" << j << ": " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "object_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*

		// Calculate the volume of each cluster
		// Create a convex hull around the cluster and calculate the total volume
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::ConvexHull<pcl::PointXYZRGB> hull;
		hull.setInputCloud (cloud_cluster);
		hull.setDimension(3);
		hull.setComputeAreaVolume(true); // This creates alot of output, but it's necessary for getTotalVolume() ....
		hull.reconstruct (*hull_points);
		std::cout << "Volume of Cluster " << j << " is: " << hull.getTotalVolume() << std::endl;
		std::cerr << "Convex hull has: " << hull_points->points.size ()
            << " data points." << std::endl;

		// Centroid calulcation
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (*hull_points, centroid);	
		std::cout << "The centroid vector is: " << centroid[0] << ","<<centroid[1]<<"," << centroid[2] <<","<< centroid[3] <<std::endl;

		// pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  	// viewer.showCloud(cloud_cluster);
		// vtkSmartPointer<vtkDataSet> data = pcl::visualization::createSphere (centroid, 20);
		// while(!viewer.wasStopped ()){}
		pcl::visualization::PCLVisualizer viewer("Cluster visualization"); 
		viewer.addCoordinateSystem(0.025); 
		viewer.addPointCloud(cloud_cluster); 
		// viewer.addSphere(pcl::PointXYZ(centroid[0],centroid[1],centroid[2]), 0.1, "sphere1"); 
		viewer.addSphere(pcl::PointXYZ(centroid[0],centroid[1],centroid[2]), 0.01, 255,0,0,"centroid"); 
		viewer.spin(); 

    std::stringstream sh;
    sh << "object_cluster_" << j << "_hull.pcd";
		writer.write(sh.str (),*hull_points);

    j++;
  }

  return (0);
}