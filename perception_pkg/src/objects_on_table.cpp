/**
 * objects_on_table.cpp
 *
 * Process a complete point cloud of the given scene
 * and writes every point above the table
 * into the object_clusters.pcd file.
 */
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

int main(int argc, char **argv)
{

	//path to pcd file needs to be given as a parameter
	if(argc<2)
	{
	        std::cerr<<"Please specify a pcd file to use"<<std::endl;
	        std::cerr<<"Run program like this: objects_on_table [filename.pcd]"<<std::endl;
		exit(0);
	}
	//objects for reading/writing pcd files
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	//point cloud objects
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	//read in point cloud from pcd file
	reader.read(argv[1],*cloud_in);
	std::cerr<<"Input cloud has " <<cloud_in->points.size()<<" point! "<<std::endl;

	//create visualization opbject
	// pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	// viewer.showCloud(cloud_in);
	// while(!viewer.wasStopped ()){}

	//removing nans from point clouds
	std::vector<int> nans;
	pcl::removeNaNFromPointCloud(*cloud_in,*cloud_nanles,nans);
	std::cerr<<"Size of point cloud after removal of nans "<<cloud_nanles->points.size()<<"!"<<std::endl;
	std::cerr<<"Writing output cloud nanless.pcd"<<std::endl;
	writer.write("nanles.pcd",*cloud_nanles);

	// //estimate normals
	// pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	// pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	// ne.setInputCloud(cloud_in);
	// ne.setSearchMethod (tree);
	// ne.setRadiusSearch (0.03);
	// ne.compute(*cloud_normals);

	// //create cloud for visualization
	// pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal_color (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	// pcl::concatenateFields(*cloud_in, *cloud_normals, *cloud_normal_color);
	// std::cerr<<"Writing output cloud normals.pcd"<<std::endl;
	// writer.write("normals.pcd",*cloud_normal_color);

	//filtering cloud on z axis
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud_nanles);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.5);
	//pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_filtered);
	std::cerr<<"Point Cloud has "<<cloud_filtered->points.size()<<" after filtering"<<std::endl;
	std::cerr<<"Writing output cloud filtered.pcd"<<std::endl;
	writer.write("filtered.pcd",*cloud_filtered);

	//voxelizing cloud
	pcl::VoxelGrid <pcl::PointXYZRGB> vg;
	vg.setInputCloud(cloud_filtered);
	vg.setLeafSize(0.01f,0.01f,0.01f);
	vg.filter(*cloud_downsampled);
	std::cerr<<"Point Cloud has "<<cloud_downsampled->points.size()<<" after downsampeling"<<std::endl;
	std::cerr<<"Writing output cloud filtered.pcd"<<std::endl;
	writer.write("downsampled.pcd",*cloud_downsampled);

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
	std::cerr<<"Plane Inliers found: "<<inliers->indices.size()<<std::endl;

	//splitting the cloud in two: plane + other
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	std::cerr<<"Extracting plane"<<std::endl;
	extract.filter(*cloud_plane);
	std::cerr<<"Writing output cloud plane.pcd"<<std::endl;
	writer.write("plane.pcd",*cloud_plane);

	// Remove the plane from the rest of the point cloud
	extract.setNegative(true);
	extract.filter(*cloud_clusters);

	std::cerr<<"Writing output cloud clusters.pcd"<<std::endl;
	writer.write("clusters.pcd",*cloud_clusters);

	// Create a ConvexHull for the table plane
  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;
  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud_projected);
  // chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);
	std::cerr<<"Writing plane hull cloud to plane_hull.pcd"<<std::endl;
	writer.write("plane_hull.pcd",*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

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
		std::cerr<<"Writing output cloud object_clusters.pcd"<<std::endl;
		writer.write("object_clusters.pcd",*object_clusters);
	}
	else
	 PCL_ERROR ("The input cloud does not represent a planar surface.\n");

	return 0;
}
