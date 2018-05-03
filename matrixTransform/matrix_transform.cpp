#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_base.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>


#include <pcl/PointIndices.h>
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
	//PointCloud<PointXYZRGBL>::Ptr cloudFiltered(new PointCloud<PointXYZRGBL>());

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
	cloud1->width = 30;
	cloud1->height = 1;
	cloud1->points.resize(cloud1->width * cloud1->height);

	cloud2->width = 50;
	cloud2->height = 1;
	cloud2->points.resize(cloud2->width * cloud2->height);

	for (size_t i = 0; i < cloud1->points.size(); ++i)
	{
		cloud1->points[i].x = 0.01f + (float)i/3;
		cloud1->points[i].y = 0.01f + (float)i/3;
		cloud1->points[i].z = 0.01f + (float)i/3;
	}

	for (size_t i = 0; i < cloud2->points.size(); ++i)
	{
		cloud2->points[i].x = 9.99f - (float)i/5;
		cloud2->points[i].y = 9.99f - (float)i/5;
		cloud2->points[i].z = 9.99f - (float)i/5;
	}
	// Iteriere ueber beide Clouds und gebe jedem Punkt jeweils Label 1 oder 2, je nach Zugehhoerigkeit
	uint32_t label1 = 1;
	for (auto &p1 : cloud1->points) p1.label = label1;

	uint32_t label2 = 2;
	for (auto &p2 : cloud2->points) p2.label = label2;

	// Merge beide Clouds (alternativ concatenatePointCloud?)
	/*cloud3->width = cloud1->width + cloud2->width;
	cloud3->height = 1;
	cloud3->points.resize(cloud3->width * cloud3->height);

	for (size_t i = 0; i < cloud1->size(); i++) {
		cloud3->points[i] = cloud1->points[i];
	}

	for (size_t i = 0; i < cloud2->size(); i++) {
		cloud3->points[i + cloud1->size()] = cloud2->points[i];
	}*/
	
	*cloud3 = *cloud1;
	*cloud3 = *cloud3 + *cloud2;

	// Tiefes des Baumes (standard scheint m, moeglicherweise immer im Bezug auf Quelldaten)
	float resolution = 1.33f;

	// Octree auf gemergte Pointcloud
	OctreePointCloud<PointXYZRGBL> octree(resolution);

	//octree.max_objs_per_leaf_ = (size_t)50000;

	octree.setInputCloud(cloud3);
	// BoundingBox muss vor addPoints ausgelesen werden, Begruendung unklar aber durch Tests bestaetigt
	// sowohl defineBoudingBox() als auch octree.defineBoundingBox(10.0f) liefern gewuenschte Resultate
	octree.defineBoundingBox();
	//octree.defineBoundingBox(10.0f);
	octree.addPointsFromInputCloud();

	OctreePointCloud<PointXYZRGBL>::LeafNodeIterator iter(&octree);

	// Vector speichert Indizes von Punkten im aktuellen Leafnode
	std::vector<int> indexVector;
	std::vector<int> indexVectorCloud1;
	std::vector<int> indexVectorCloud2;
	/*PointIndices::Ptr indexVectorCloud1(new pcl::PointIndices());
	PointIndices::Ptr indexVectorCloud2(new pcl::PointIndices());*/
	// Aus jedem Leafnode werden die relevanten Nodes in gesamtIndices gespeichert
	//PointIndices::Ptr gesamtIndices(new pcl::PointIndices());
	std::vector<int> gesamtIndices;

	// Iteriere ueber alle Leafnodes
	for (iter = octree.leaf_begin(); iter != octree.leaf_end(); ++iter) 
		{

			indexVector = iter.getLeafContainer().getPointIndicesVector();

			/*int counterCloud1;
			int counterCloud2;*/

			// Ueberpruefe bei jedem Punkt im Leafnode, welches Label er hat
			// Die Pointcloud mit mehr Punkten im Leafnode wird praeferiert
			// Zu Testzwecken erstmal einfach Ausgabe der Punkte im Vektor
			for (size_t i = 0; i < indexVector.size(); ++i)
			{
				std::cout << " x " << cloud3->points[indexVector[i]].x
					<< " Size " << indexVector.size()
					<< " Label " << cloud3->points[indexVector[i]].label
					<< std::endl;

				int counterCloud1 = 0;
				int counterCloud2 = 0;

				if (cloud3->points[indexVector[i]].label == 1) {
					counterCloud1++;
					// jeden index aus Cloud1 in indexVectorCloud1 hinzufuegen
					indexVectorCloud1.push_back(indexVector[i]);
				}
				else {
					counterCloud2++;
					// analog fuer Cloud2
					indexVectorCloud2.push_back(indexVector[i]);
				}
			}

			// fuege neue Punkte hinten an
			int temp = gesamtIndices.size();
			std::cout << " Cloud1 " << indexVectorCloud1.size() << " Cloud2 " << indexVectorCloud2.size();
			// aktuelle Cloud1 Punkte adden zu relevantem Punkte Pool
			if (indexVectorCloud1.size() >= indexVectorCloud2.size()) {
				for (int i = 0; i < indexVectorCloud1.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud1[i]);
				}
			}

			// aktuelle Cloud2 Punkte adden zu relevantem Punkte Pool
			else {
				for (size_t i = 0; i < indexVectorCloud2.size(); ++i) {
					gesamtIndices.push_back(indexVectorCloud2[i]);
				}
			}
			std::cout << " Ende eines Leafnode " << std::endl;
			indexVectorCloud1.clear();
			indexVectorCloud2.clear();
		}
		for (size_t i = 0; i < gesamtIndices.size(); ++i) {
			std::cout << "    " << cloud3->points[gesamtIndices[i]].x <<
						 "    " << cloud3->points[gesamtIndices[i]].label << 
						 "    " << gesamtIndices[i] << std::endl;
		}

	return 0;
}