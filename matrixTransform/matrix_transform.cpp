#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_base.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/PointIndices.h>
#include <inttypes.h>
#include <stdint.h>

// je nach gewünschter Methode boolean-Variablen auf true setzten
bool RemoveOutlier = false; // Punkte muessen Anzahl n Nachbarn in Radius r besitzen, um nicht gefiltert zu werden
bool ConditionalRemoval = false; // Custom condition nach der gefiltert wird !NICHT IMPLEMENTIERT!
bool StatistOutlier = false; // Removed noisy measurement (outliers) mit statistical analysis
bool MovingLeastSquare = false; // Smoothen und resample noisy data via polynomial interpolation (runtime >8min, verliert RGB daten)
bool PairWiseIncrementalRegistration = false; // braucht erst Implementierung der restlichen Scan Standorte
bool NormalDistributionTrans = true; // braucht zwei Clouds die man alignen moechte, waere also wenn eher alternative zu bisher

using namespace pcl;
using namespace octree;
using namespace io;

// PairWiseIncrementalRegistration stuff
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

visualization::PCLVisualizer *p;
//linker und rechter viewport (des visualizers)
int vp_1, vp_2;

struct PLY
{
	PointCloud<PointXYZRGB>::Ptr cloud;
	std::string f_name;

	PLY() : cloud(new PointCloud<PointXYZRGB>) {};
};

struct PLYComparator
{
	bool operator () (const PLY& p1, const PLY& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public PointRepresentation <PointNormal>
{
	using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormal &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
*
*/
void showCloudsLeft(const PointCloud<PointXYZRGB>::Ptr cloud_target, const PointCloud<PointXYZRGB>::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointXYZRGB> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZRGB> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

	PCL_INFO("Press q to begin the registration.\n");
	p->spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
*
*/
void showCloudsRight(const PointCloud<PointNormal>::Ptr cloud_target, const PointCloud<PointNormal>::Ptr cloud_source)
{
	p->removePointCloud("source");
	p->removePointCloud("target");


	PointCloudColorHandlerGenericField<PointNormal> tgt_color_handler(cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormal> src_color_handler(cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");


	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
* \param argc the number of arguments (pass from main ())
* \param argv the actual command line arguments (pass from main ())
* \param models the resultant vector of point cloud datasets
*/
void loadData(int argc, char **argv, std::vector<PLY, Eigen::aligned_allocator<PLY> > &models)
{
	std::string extension(".ply");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string(argv[i]);
		// Needs to be at least 5: .plot
		if (fname.size() <= extension.size())
			continue;

		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

		//check that the argument is a ply file
		if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
		{
			// Load the cloud and saves it into the global list of models
			PLY m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile(argv[i], *m.cloud);
			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

			models.push_back(m);
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
* \param cloud_src the source PointCloud
* \param cloud_tgt the target PointCloud
* \param output the resultant aligned source PointCloud
* \param final_transform the resultant transform between source and target
*/
void pairAlign(const PointCloud<PointXYZRGB>::Ptr cloud_src, const PointCloud<PointXYZRGB>::Ptr cloud_tgt, PointCloud<PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud<PointXYZRGB>::Ptr src(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr tgt(new PointCloud<PointXYZRGB>);
	pcl::VoxelGrid<PointXYZRGB> grid;
	if (downsample)
	{
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloud<PointNormal>::Ptr points_with_normals_src(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr points_with_normals_tgt(new PointCloud<PointNormal>);

	pcl::NormalEstimation<PointXYZRGB, PointNormal> norm_est;
	pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(0.1);
	// Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);



	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloud<PointNormal>::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerCustom<PointXYZRGB> cloud_tgt_h(output, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZRGB> cloud_src_h(cloud_src, 255, 0, 0);
	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO("Press q to continue the registration.\n");
	p->spin();

	p->removePointCloud("source");
	p->removePointCloud("target");

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}
// Ende pairwiseincremental stuff (vor main teil)


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
	// Teil main pairwiseincremental
	if (PairWiseIncrementalRegistration) 
	{
		// Load data
		std::vector<PLY, Eigen::aligned_allocator<PLY> > data;
		loadData(argc, argv, data);

		// Check user input
		if (data.empty())
		{
			PCL_ERROR("Syntax is: %s <source.ply> <target.ply> [*]", argv[0]);
			PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
			return (-1);
		}
		PCL_INFO("Loaded %d datasets.", (int)data.size());

		// Create a PCLVisualizer object
		p = new visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
		p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
		p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

		PointCloud<PointXYZRGB>::Ptr result(new PointCloud<PointXYZRGB>), source, target;
		Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

		for (size_t i = 1; i < data.size(); ++i)
		{
			source = data[i - 1].cloud;
			target = data[i].cloud;

			// Add visualization data
			showCloudsLeft(source, target);

			PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>);
			PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
			pairAlign(source, target, temp, pairTransform, true);

			//transform current pair into the global transform
			transformPointCloud(*temp, *result, GlobalTransform);

			//update the global transform
			GlobalTransform = GlobalTransform * pairTransform;

			//save aligned pair, transformed into the first cloud's frame
			std::stringstream ss;
			ss << i << ".ply";
			savePLYFile(ss.str(), *result, true);
			//io::savePLYFileBinary(ss.str(), *result)
		}
	}
	// Ende Maintail pairwiseincremental

	
	else 
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
		PointCloud<PointXYZRGB>::Ptr cloudXYZ6(new PointCloud<PointXYZRGB>());
		PointCloud<PointXYZRGB>::Ptr cloudXYZ7(new PointCloud<PointXYZRGB>());
		PointCloud<PointXYZRGB>::Ptr cloudXYZ8(new PointCloud<PointXYZRGB>());
		PointCloud<PointXYZRGB>::Ptr cloudXYZ9(new PointCloud<PointXYZRGB>());
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
			if (PairWiseIncrementalRegistration && filenames.size() == 9) {
				if (io::loadPCDFile(argv[filenames[2]], *cloudXYZ3) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[2]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPCDFile(argv[filenames[3]], *cloudXYZ4) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[3]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPCDFile(argv[filenames[4]], *cloudXYZ5) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[4]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPCDFile(argv[filenames[5]], *cloudXYZ6) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[5]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPCDFile(argv[filenames[6]], *cloudXYZ7) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[6]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPCDFile(argv[filenames[7]], *cloudXYZ8) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[7]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPCDFile(argv[filenames[8]], *cloudXYZ9) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[8]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
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
			if (PairWiseIncrementalRegistration && filenames.size() == 9) {
				if (io::loadPLYFile(argv[filenames[2]], *cloudXYZ3) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[2]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPLYFile(argv[filenames[3]], *cloudXYZ4) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[3]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPLYFile(argv[filenames[4]], *cloudXYZ5) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[4]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPLYFile(argv[filenames[5]], *cloudXYZ6) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[5]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPLYFile(argv[filenames[6]], *cloudXYZ7) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[6]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPLYFile(argv[filenames[7]], *cloudXYZ8) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[7]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
				if (io::loadPLYFile(argv[filenames[8]], *cloudXYZ9) < 0) {
					std::cout << "Fehler beim Laden der Cloud " << argv[filenames[8]] << std::endl << std::endl;
					showHelp(argv[0]);
					return -1;
				}
			}
		}

		// Konvertiere die beiden eingelesenen Clouds in (neue) XYZRGBL Clouds
		copyPointCloud(*cloudXYZ1, *cloudFarbig1);
		copyPointCloud(*cloudXYZ2, *cloudFarbig2);

		// Initialisiere RotBlau Cloud
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

		if (NormalDistributionTrans == false) 
		// siehe Kommentare (unnoetige Berechnungen vermeiden)
		{ 
			*cloudMergedFarbig = *cloudFarbig1;
			*cloudMergedFarbig = *cloudMergedFarbig + *cloudFarbig2;

			*cloudMergedRotBlau = *cloudRotBlau1;
			*cloudMergedRotBlau = *cloudMergedRotBlau + *cloudRotBlau2;

			// Tiefes des Baumes (standard scheint m, moeglicherweise immer im Bezug auf Quelldaten)
			float resolution = 0.10f;

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
			std::string writePathFarbig = "Farbig" + s + ".ply";
			std::string writePathRotBlau = "RotBlau" + s + ".ply";
			io::savePLYFileBinary(writePathFarbig, *cloudFilteredFarbig);
			io::savePLYFileBinary(writePathRotBlau, *cloudFilteredRotBlau);
		

			//////////////////////////////////
			// Optionale "Nachfilterungen" //
			////////////////////////////////
			if (RemoveOutlier)
			{
				PointCloud<PointXYZRGBL>::Ptr cloudFilteredFarbigRemOut(new PointCloud<PointXYZRGBL>());
				RadiusOutlierRemoval<PointXYZRGBL> RemOut;

				RemOut.setInputCloud(cloudFilteredFarbig);
				RemOut.setRadiusSearch(0.02);
				RemOut.setMinNeighborsInRadius(2);
				RemOut.filter(*cloudFilteredFarbigRemOut);

				writePathFarbig = "FarbigRemOut" + s + ".ply";
				savePLYFileBinary(writePathFarbig, *cloudFilteredFarbigRemOut);
			}

			if (StatistOutlier)
			{
				PointCloud<PointXYZRGBL>::Ptr cloudFilteredFarbigStatOut(new PointCloud<PointXYZRGBL>());
				StatisticalOutlierRemoval<pcl::PointXYZRGBL> StatOr;

				StatOr.setInputCloud(cloudFilteredFarbig);
				StatOr.setMeanK(50);
				StatOr.setStddevMulThresh(1.0);
				StatOr.filter(*cloudFilteredFarbigStatOut);

				writePathFarbig = "FarbigStatOut" + s + ".ply";
				savePLYFileBinary(writePathFarbig, *cloudFilteredFarbigStatOut);
			}

			// sagt explizit in der Dokumentation, dass auch andere Punkttypen als XYZ vertraeglich sind, 
			// vertraegt aber kein XYZRGBL, LOL
			if (MovingLeastSquare)
			{
				PointCloud<PointXYZRGB>::Ptr cloudFilteredFarbigNoL(new PointCloud<PointXYZRGB>());
				copyPointCloud(*cloudFilteredFarbig, *cloudFilteredFarbigNoL);
				search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);

				PointCloud<PointNormal> mls_points;
				MovingLeastSquares<PointXYZRGB, PointNormal> mls;
				mls.setComputeNormals(true);

				mls.setInputCloud(cloudFilteredFarbigNoL);
				mls.setPolynomialFit(true);
				mls.setSearchMethod(tree);
				mls.setSearchRadius(0.03);

				mls.process(mls_points);

				writePathFarbig = "FarbigMLS" + s + ".ply";
				//io::savePCDFile(writePathFarbig, mls_points);
				savePLYFileBinary(writePathFarbig, mls_points);
			}
		}

		if (NormalDistributionTrans)
		{
			/*PointCloud<PointXYZRGBL>::Ptr filtered_cloud(new PointCloud<PointXYZRGBL>);
			ApproximateVoxelGrid<PointXYZRGBL> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
			approximate_voxel_filter.setInputCloud(cloudFarbig1);
			approximate_voxel_filter.filter(*filtered_cloud);*/

			NormalDistributionsTransform<PointXYZRGBL, PointXYZRGBL> ndt;
			ndt.setTransformationEpsilon(0.005);
			ndt.setStepSize(0.05);
			ndt.setResolution(0.25);

			ndt.setMaximumIterations(100);

			/*ndt.setInputSource(filtered_cloud);*/
			ndt.setInputSource(cloudFarbig1);
			ndt.setInputTarget(cloudFarbig2);

			Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
			Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
			Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

			PointCloud<PointXYZRGBL>::Ptr output_cloud(new PointCloud<PointXYZRGBL>);
			ndt.align(*output_cloud, init_guess);

			transformPointCloud(*cloudFarbig1, *output_cloud, ndt.getFinalTransformation());

			// Resolution eines Voxels
			float resolution = ndt.getResolution();

			// Schreibe cloudFilteredFarbig und cloudFilteredRotBlau jeweils in eine .ply Datei
			std::ostringstream ss;
			ss << resolution;
			std::string s(ss.str());
			std::string writePathFarbig = "FarbigNDT" + s + ".ply";
			savePLYFileBinary(writePathFarbig, *output_cloud);
		}

		return 0;
	}
}