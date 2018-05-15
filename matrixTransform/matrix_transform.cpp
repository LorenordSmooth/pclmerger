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

void
showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:  Show this help." << std::endl;
}

int
main(int argc, char** argv)
{

	// Hilfe Funktion
	if (console::find_switch(argc, argv, "-h") || console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
		return 0;
	}

	// Einlesen der Dateinamen
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = console::parse_file_extension_argument(argc, argv, ".ply");

	if (filenames.size() > 2) {
		filenames = console::parse_file_extension_argument(argc, argv, ".pcd");

		if (filenames.size() > 2) {
			showHelp(argv[0]);
			return -1;
		}
		else {
			file_is_pcd = true;
		}
	}

	// Cloud XYZ1&2 sind die geladenen XYZRGB Baseclouds, Cloud1&2 converted zu XYZRGBL, CloudMerged die gemergte, Cloud4 die "gefilterte"
	PointCloud<PointXYZRGB>::Ptr cloudXYZ1(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGB>::Ptr cloudXYZ2(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGB>::Ptr cloudXYZ3(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGB>::Ptr cloudXYZ4(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGB>::Ptr cloudXYZ5(new PointCloud<PointXYZRGB>());
	PointCloud<PointXYZRGBL>::Ptr cloudFarbig1(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudFarbig2(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudRotBlau1(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudRotBlau2(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud3(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud4(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloud5(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudMergedFarbig(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudMergedRotBlau(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudFilteredFarbig(new PointCloud<PointXYZRGBL>());
	PointCloud<PointXYZRGBL>::Ptr cloudFilteredRotBlau(new PointCloud<PointXYZRGBL>());

	// Beide Clouds sind entweder .ply oder .pcd
	if (file_is_pcd) {
		if (io::loadPCDFile(argv[filenames[0]], *cloudXYZ1) < 0) {
			std::cout << "Fehler beim Laden der Cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPCDFile(argv[filenames[1]], *cloudXYZ2) < 0) {
			std::cout << "Fehler beim Laden der Cloud " << argv[filenames[1]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	else {
		if (io::loadPLYFile(argv[filenames[0]], *cloudXYZ1) < 0) {
			std::cout << "Fehler beim Laden der Cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
		if (io::loadPLYFile(argv[filenames[1]], *cloudXYZ2) < 0) {
			std::cout << "Fehler beim Laden der Cloud " << argv[filenames[1]] << std::endl << std::endl;
			showHelp(argv[0]);
			return -1;
		}
	}
	
	//const std::vector<PointXYZRGB,Eigen::aligned_allocator<PointXYZRGB>> test = cloudXYZ1->points;

	/*for (size_t i = 0; i < cloudXYZ1->points.size(); ++i) {
		cloudXYZ1->points[i].x = cloud1->points[i].x;
		cloudXYZ1->points[i].y = cloud1->points[i].y;
		cloudXYZ1->points[i].z = cloud1->points[i].z;
		cloudXYZ1->points[i].rgb = cloud1->points[i].rgb;
	}*/

	// Konvertiere die beiden eingelesenen Clouds in (neue) XYZRGBL Clouds
	copyPointCloud(*cloudXYZ1, *cloudFarbig1);
	copyPointCloud(*cloudXYZ2, *cloudFarbig2);

	/*for (size_t i = 0; i < cloudXYZ1->points.size(); ++i)
		copyPoint(cloudXYZ1->points[i], cloud1->points[i]);*/

	/*for (size_t i = 0; i < cloudXYZ2->points.size(); ++i)
		copyPoint(cloudXYZ2->points[i], cloud2->points[i]);*/

	// Zu Testzwecken kleine Pointclouds:
	/*cloud1->width = 30;
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
	}*/

	*cloudRotBlau1 = *cloudFarbig1;
	*cloudRotBlau2 = *cloudFarbig2;

	// Iteriere ueber beide Clouds und gebe jedem Punkt jeweils Label 1 oder 2, je nach Zugehhoerigkeit
	uint32_t label1 = 1;
	uint8_t r1 = 255, g1 = 0, b1 = 0;
	uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);

	// Labels und Farben setzten
	for (auto &p1 : cloudFarbig1->points) {
		p1.label = label1;
	}
	for (auto &p1 : cloudRotBlau1->points) {
		p1.label = label1;
		p1.rgb = *reinterpret_cast<float*>(&rgb1);
	}

	uint32_t label2 = 2;
	uint8_t r2 = 0, g2 = 0, b2 = 255; 
	uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
	for (auto &p2 : cloudFarbig2->points) {
		p2.label = label2;
	}
	for (auto &p2 : cloudRotBlau2->points) {
		p2.label = label2;
		p2.rgb = *reinterpret_cast<float*>(&rgb2);
	}
	// Merge beide Clouds (alternativ concatenatePointCloud?)
	/*cloudMerged->width = cloud1->width + cloud2->width;
	cloudMerged->height = 1;
	cloudMerged->points.resize(cloudMerged->width * cloudMerged->height);

	for (size_t i = 0; i < cloud1->size(); i++) {
		cloudMerged->points[i] = cloud1->points[i];
	}

	for (size_t i = 0; i < cloud2->size(); i++) {
		cloudMerged->points[i + cloud1->size()] = cloud2->points[i];
	}*/
	
	*cloudMergedFarbig = *cloudFarbig1;
	*cloudMergedFarbig = *cloudMergedFarbig + *cloudFarbig2;

	*cloudMergedRotBlau = *cloudRotBlau1;
	*cloudMergedRotBlau = *cloudMergedRotBlau + *cloudRotBlau2;

	// Tiefes des Baumes (standard scheint m, moeglicherweise immer im Bezug auf Quelldaten)
	float resolution = 0.2f;

	// Octree auf gemergte Pointcloud
	OctreePointCloud<PointXYZRGBL> octreeFarbig(resolution);
	OctreePointCloud<PointXYZRGBL> octreeRotBlau(resolution);

	//octree.max_objs_per_leaf_ = (size_t)50000;
	
	octreeFarbig.setInputCloud(cloudMergedFarbig);
	octreeRotBlau.setInputCloud(cloudMergedRotBlau);
	// BoundingBox muss vor addPoints ausgelesen werden, Begruendung unklar aber durch Tests bestaetigt
	// sowohl defineBoudingBox() als auch octree.defineBoundingBox(10.0f) liefern gewuenschte Resultate
	octreeFarbig.defineBoundingBox();
	octreeRotBlau.defineBoundingBox();
	//octree.defineBoundingBox(10.0f);
	octreeFarbig.addPointsFromInputCloud();
	octreeRotBlau.addPointsFromInputCloud();

	OctreePointCloud<PointXYZRGBL>::LeafNodeIterator iterFarbig(&octreeFarbig);
	OctreePointCloud<PointXYZRGBL>::LeafNodeIterator iterRotBlau(&octreeRotBlau);

	// Vector speichert Indizes von Punkten im aktuellen Leafnode
	std::vector<int> indexVector;
	std::vector<int> indexVectorCloud1;
	std::vector<int> indexVectorCloud2;
	// Aus jedem Leafnode werden die relevanten Nodes in gesamtIndices gespeichert
	std::vector<int> gesamtIndices;

	// Iteriere ueber alle Leafnodes
	for (iterFarbig = octreeFarbig.leaf_begin(); iterFarbig != octreeFarbig.leaf_end(); ++iterFarbig)
		{

			indexVector = iterFarbig.getLeafContainer().getPointIndicesVector();

			// Ueberpruefe bei jedem Punkt im Leafnode, welches Label er hat
			// Die Pointcloud mit mehr Punkten im Leafnode wird praeferiert
			// Zu Testzwecken erstmal einfach Ausgabe der Punkte im Vektor
			for (size_t i = 0; i < indexVector.size(); ++i)
			{
				/*std::cout << " x " << cloudMerged->points[indexVector[i]].x
					<< " Size " << indexVector.size()
					<< " Label " << cloudMerged->points[indexVector[i]].label
					<< std::endl;*/

				int counterCloud1 = 0;
				int counterCloud2 = 0;

				if (cloudMergedFarbig->points[indexVector[i]].label == 1) {
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
			indexVectorCloud1.clear();
			indexVectorCloud2.clear();
		}

	// gesamtIndices erhaelt nun die gefilterte cloudMergedFarbig, diese Indices gilt es nun wieder in eine Cloud zu ueberfuehren
	for (size_t i = 0; i < gesamtIndices.size(); ++i) {
		cloudFilteredFarbig->push_back(cloudMergedFarbig->points[gesamtIndices[i]]);
	}

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// !!! BEGINN ZWEITE LOOP ROT BLAU !!!
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// Identische Loop fuer RotBlau merged Wolke
	for (iterRotBlau = octreeRotBlau.leaf_begin(); iterRotBlau != octreeRotBlau.leaf_end(); ++iterRotBlau)
	{

		indexVector = iterRotBlau.getLeafContainer().getPointIndicesVector();

		// Ueberpruefe bei jedem Punkt im Leafnode, welches Label er hat
		// Die Pointcloud mit mehr Punkten im Leafnode wird praeferiert
		// Zu Testzwecken erstmal einfach Ausgabe der Punkte im Vektor
		for (size_t i = 0; i < indexVector.size(); ++i)
		{
			int counterCloud1 = 0;
			int counterCloud2 = 0;

			if (cloudMergedRotBlau->points[indexVector[i]].label == 1) {
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
		indexVectorCloud1.clear();
		indexVectorCloud2.clear();
	}
	// gesamtIndices erhaelt nun die gefilterte cloudMergedRotBlau, diese Indices gilt es nun wieder in eine Cloud zu ueberfuehren
	for (size_t i = 0; i < gesamtIndices.size(); ++i) {
		cloudFilteredRotBlau->push_back(cloudMergedRotBlau->points[gesamtIndices[i]]);
	}

	// Uebersicht zu Testzwecken
	/*for (size_t i = 0; i < gesamtIndices.size(); ++i) {
			std::cout << "    " << cloudMerged->points[gesamtIndices[i]].x <<
						 "    " << cloudMerged->points[gesamtIndices[i]].label << 
						 "    " << gesamtIndices[i] << std::endl;
	}*/

	// Schreibe cloudFilteredFarbig und cloudFilteredRotBlau jeweils in eine .ply Datei
	std::ostringstream ss;
	ss << resolution;
	std::string s(ss.str());
	std::string writePathFarbig = "Farbig"+s+".ply";
	std::string writePathRotBlau = "RotBlau"+s+".ply";
	io::savePLYFileBinary(writePathFarbig, *cloudFilteredFarbig);
	io::savePLYFileBinary(writePathRotBlau, *cloudFilteredRotBlau);

	return 0;
}