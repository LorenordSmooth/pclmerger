#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_base.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <inttypes.h>
#include <stdint.h>

using namespace pcl;
using namespace octree;

// This function displays the help
//void
//showHelp(char * program_name)
//{
//	std::cout << std::endl;
//	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
//	std::cout << "-h:  Show this help." << std::endl;
//}

// This is the main function
int
main(int argc, char** argv)
{

	// Show help
	//if (console::find_switch(argc, argv, "-h") || console::find_switch(argc, argv, "--help")) {
	//	showHelp(argv[0]);
	//	return 0;
	//}

	//// Fetch point cloud filename in arguments | Works with PCD and PLY files
	//std::vector<int> filenames;
	//bool file_is_pcd = false;

	//filenames = console::parse_file_extension_argument(argc, argv, ".ply");

	//if (filenames.size() != 1) {
	//	filenames = console::parse_file_extension_argument(argc, argv, ".pcd");

	//	if (filenames.size() != 1) {
	//		showHelp(argv[0]);
	//		return -1;
	//	}
	//	else {
	//		file_is_pcd = true;
	//	}
	//}

	// Load files | Works with PCD and PLY files
	PointCloud<PointXYZRGBL>::Ptr cloud1(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud2(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud3(new PointCloud<PointXYZRGBL>());

	// Assuming both pointclouds will either be .ply or .pcd
	/*if (file_is_pcd) {
		if (io::loadPCDFile(argv[filenames[0]], *cloud1) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPCDFile(argv[filenames[1]], *cloud2) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[1]] << std::endl << std::endl;
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
		if (io::loadPLYFile(argv[filenames[1]], *cloud2) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[1]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}*/

	//Zu Testzwecken kleine Pointclouds:
	cloud1->width = 10;
	cloud1->height = 1;
	cloud1->points.resize(cloud1->width * cloud1->height);

	cloud2->width = 10;
	cloud2->height = 1;
	cloud2->points.resize(cloud2->width * cloud2->height);

	for (size_t i = 0; i < cloud1->points.size(); ++i)
	{
		cloud1->points[i].x = 9.99f - i;
		cloud1->points[i].y = 9.99f - i;
		cloud1->points[i].z = 9.99f - i;
	}

	for (size_t i = 0; i < cloud2->points.size(); ++i)
	{
		cloud1->points[i].x = 10.01f - i;
		cloud1->points[i].y = 10.01f - i;
		cloud1->points[i].z = 10.01f - i;
	}
	// Iteriere ueber beide Clouds und gebe jedem Punkt jeweils Label 1 oder 2, je nach Zugehhoerigkeit
	uint32_t label1 = 1;
	for (auto &p1 : cloud1->points) p1.label = label1;

	uint32_t label2 = 2;
	for (auto &p2 : cloud2->points) p2.label = label2;

	// Merge beide Clouds (alternativ concatenatePointCloud?)
	*cloud3 = *cloud1;
	*cloud3 = *cloud3 + *cloud2;


	// Tiefes des Baumes (standard scheint m, moeglicherweise immer im Bezug auf Quelldaten)
	float resolution = 128.0f;

	// Octree auf gemergte Pointcloud
	OctreePointCloud<PointXYZRGBL> octree(resolution);

	//octree.max_objs_per_leaf_ = (size_t)50000;

	octree.setInputCloud(cloud3);
	octree.addPointsFromInputCloud();

	octree.defineBoundingBox(); // startet an erstem eingelesen Punkt, kann also sehr ungenau sein
								
	OctreePointCloud<PointXYZRGBL>::LeafNodeIterator iter(&octree);

	// Vector speichert Indizes von Punkten im aktuellen Leafnode
	std::vector<int> indexVector;

	// Iteriere ueber alle Leafnodes
	for (iter = octree.leaf_begin(); iter != octree.leaf_end(); ++iter) 
		{

		indexVector = iter.getLeafContainer().getPointIndicesVector();

		// Ueberpruefe bei jedem Punkt im Leafnode, welches Label er hat
		// Die Pointcloud mit mehr Punkten im Leafnode wird praeferiert
		// Zu Testzwecken erstmal einfach Ausgabe der Punkte im Vektor
		for (size_t i = 0; i < indexVector.size(); ++i)
			std::cout << "    " << cloud3->points[indexVector[i]].x
			<< " " << cloud3->points[indexVector[i]].y
			<< " " << cloud3->points[indexVector[i]].z 
			<< " " << cloud3->points[indexVector[i]].label
			<< " Ende Leafnode " << std::endl;


	//	std::vector<int>::iterator iterVector;
	//	for (iterVector = indexVector.begin;
	//		iterVector != indexVector.end; ++iterVector) {
	//		std::cout << "    " << cloud1->points[indexVector[iterVector]].x
	//			<< " " << cloud1->points[indexVector[iterVector]].y
	//			<< " " << cloud1->points[indexVector[iterVector]].z << std::endl;
	//		/*int counterCloud1;
	//		int counterCloud2;
	//		if (cloud3->points[iterVector].label = 1) {
	//			counterCloud1++;
	//		}
	//		else {
	//			counterCloud2++;
	//		}
	//	}*/
			}

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