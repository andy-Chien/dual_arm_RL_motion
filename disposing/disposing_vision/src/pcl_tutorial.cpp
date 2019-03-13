#include "pcl_tutorial.h"

Pcl_tutorial::Pcl_tutorial()
{

}
Pcl_tutorial::~Pcl_tutorial(){
    
}

void Pcl_tutorial::cylinder_segmentation(pcl::PointCloud<PointT>::Ptr cloud
                                        ,pcl::PointCloud<PointT>::Ptr cloud_cylinder
                                        ,pcl::PointCloud<PointT>::Ptr cloud_plane)
{
// All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
  }  
}

void Pcl_tutorial::passthrough(pcl::PointCloud<PointTRGB>::Ptr cloud
                              ,pcl::PointCloud<PointTRGB>::Ptr cloud_filtered
                              ,char* direction,float coordinate_min, float coordinate_Max)
{
  std::cerr << "Cloud before filtering size: " << cloud->points.size () << std::endl;

  // Create the filtering object
  pcl::PassThrough<PointTRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (direction);
  pass.setFilterLimits (coordinate_min, coordinate_Max);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << cloud_filtered->points.size () << std::endl;
}

void Pcl_tutorial::calculate_normal(pcl::PointCloud<PointTRGB>::Ptr point_cloud_ptr
                                   ,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals1)
{
  pcl::NormalEstimation<PointTRGB, pcl::PointNormal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<PointTRGB>::Ptr tree (new pcl::search::KdTree<PointTRGB> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.01);
  ne.compute (*cloud_normals1);
}

void Pcl_tutorial::downsampling(pcl::PointCloud<PointTRGB>::Ptr cloud
                              , pcl::PointCloud<PointTRGB>::Ptr cloud_filtered
                              , float range)
{
  pcl::VoxelGrid<PointTRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (range,range,range);
  sor.filter (*cloud_filtered);
}
