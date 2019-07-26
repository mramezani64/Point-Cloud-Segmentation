#include <iostream>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_sampled_out (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  std::vector<pcl::PointIndices> clusters;
  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("cloud_003.pcd", *cloud_in);

  std::cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height << " data points." << std::endl;

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
  // Down-sampling to get uniform points distribution.
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize (0.08f, 0.08f, 0.08f);
  sor.filter(*cloud_sampled);
  pcl::copyPointCloud(*cloud_sampled, *cloud_sampled_out);

  // Normals extraction.
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree =
  boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal>> (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_sampled_out);
  normal_estimator.setKSearch(30);
  normal_estimator.compute(*normals);
  pcl::copyPointCloud(*normals, *cloud_sampled_out);
  // Region Growing Planes Extraction (segmentation)
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  reg.setMinClusterSize(15);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(15);
  reg.setInputCloud(cloud_sampled_out);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  reg.extract(clusters);

  // Color output cloud
 srand (time(NULL));
 for (int i = 0; i < clusters.size(); i++)
 {
   // Color segment
   uint8_t r = (rand() % 256);
   uint8_t g = (rand() % 256);
   uint8_t b = (rand() % 256);
   int32_t rgb = (r << 16) | (g << 8) | b;
   for (size_t i_point = 0; i_point < clusters[i].indices.size(); i_point++)
   {
     int idx = clusters[i].indices[i_point];
     cloud_sampled_out->points[idx].rgb = rgb;
   }
 }
  
  // Write the segments to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGBNormal> ("table_scene_lms400_segments.pcd", *cloud_sampled_out, false);

  return (0);
}
