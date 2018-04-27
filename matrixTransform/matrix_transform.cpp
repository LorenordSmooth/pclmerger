#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <inttypes.h>
#include <stdint.h>

using namespace pcl;
using namespace octree;

// This function displays the help
void
showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int
main(int argc, char** argv)
{

	// Show help
	if (console::find_switch(argc, argv, "-h") || console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
		return 0;
	}

	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = console::parse_file_extension_argument(argc, argv, ".ply");

	if (filenames.size() != 1) {
		filenames = console::parse_file_extension_argument(argc, argv, ".pcd");

		if (filenames.size() != 1) {
			showHelp(argv[0]);
			return -1;
		}
		else {
			file_is_pcd = true;
		}
	}

	// Load files | Works with PCD and PLY files
	PointCloud<PointXYZRGBL>::Ptr cloud1(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud2(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud3(new PointCloud<PointXYZRGBL>());

	// Assuming both pointclouds will either be .ply or .pcd
	if (file_is_pcd) {
		if (io::loadPCDFile(argv[filenames[0]], *cloud1) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPCDFile(argv[filenames[0]], *cloud2) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	else {
		if (io::loadPLYFile(argv[filenames[0]], *cloud1) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPLYFile(argv[filenames[0]], *cloud2) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}

	// Iteriere ueber beide Clouds und gebe jedem Punkt jeweils Label 1 oder 2, je nach Zugehhoerigkeit
	uint32_t label1 = 1;
	for (auto &p1 : cloud1->points) p1.label = label1;

	uint32_t label2 = 1;
	for (auto &p2 : cloud2->points) p2.label = label2;

	// Merge beide Clouds (alternativ concatenatePointCloud?)
	*cloud3 = *cloud1;
	*cloud3 = *cloud3 + *cloud2;


	// Tiefes des Baumes (standard scheint m, moeglicherweise immer im Bezug auf Quelldaten)
	float resolution = 0.1f;

	// Octree auf gemergte Pointcloud
	OctreePointCloudSearch<PointXYZRGBL> octree(resolution);

	octree.defineBoundingBox(); // startet an erstem eingelesen Punkt, kann also sehr ungenau sein
	//octree.max_objs_per_leaf_ = (size_t)50000;

	octree.setInputCloud(cloud3);
	octree.addPointsFromInputCloud();

	OctreeLeafNodeIterator<OctreePointCloudSearch<PointXYZRGBL>> leafIterator(&octree);

	for (leafIterator = octree.leaf_begin();
		leafIterator != octree.leaf_end(); ++leafIterator)

	/*PointXYZRGBL searchPoint;

	searchPoint.x = 38.02509;
	searchPoint.y = 138.52509;
	searchPoint.z = 666.81418;

	std::vector<int> pointIdxVec;

	if (octree1.voxelSearch(searchPoint, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z << ")"
			<< std::endl;

		for (size_t i = 0; i < pointIdxVec.size(); ++i)
			std::cout << "    " << cloud1->points[pointIdxVec[i]].x
			<< " " << cloud1->points[pointIdxVec[i]].y
			<< " " << cloud1->points[pointIdxVec[i]].z << std::endl;
	}*/

	//KdTreeFLANN<PointXYZ> kdtree;

	//kdtree.setInputCloud(cloud);

	//PointXYZ searchPoint;

	////starting searchPoint
	//searchPoint.x = 38.01541;
	//searchPoint.y = 138.50537;
	//searchPoint.z = 666.83116;

	//// K nearest neighbor search

	//int K = 10;

	//std::vector<int> pointIdxNKNSearch(K);
	//std::vector<float> pointNKNSquaredDistance(K);

	//if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	//{
	//	for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
	//		std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
	//		<< " " << cloud->points[pointIdxNKNSearch[i]].y
	//		<< " " << cloud->points[pointIdxNKNSearch[i]].z
	//		<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	//}

	return 0;
}