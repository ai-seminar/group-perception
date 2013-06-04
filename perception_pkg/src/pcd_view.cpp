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
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>

int main(int argc, char **argv)
{

	//path to pcd file needs to be given as a parameter
	if(argc<2)
	{
	        std::cerr<<"Please specify a pcd file to use"<<std::endl;
	        std::cerr<<"Run program like this: pcd_processor [filename.pcd]"<<std::endl;
		exit(0);
	}
	//objects for reading/writing pcd files
	pcl::PCDReader reader;

	//point cloud objects
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());

	//read in point cloud from pcd file
	reader.read(argv[1],*cloud_in);
	std::cerr<<"Input cloud has " <<cloud_in->points.size()<<" point! "<<std::endl;

	//create visualization opbject
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud_in);
	while(!viewer.wasStopped ()){}

	return 0;
}
