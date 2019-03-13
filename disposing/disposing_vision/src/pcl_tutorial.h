#define PCL_TUTORIAL_H
#ifdef PCL_TUTORIAL_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string>

//cylinder_segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//passthrough
#include <pcl/filters/passthrough.h>

//PCLVisualizer
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//Downsampling
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;

class Pcl_tutorial
{
private:

public:
    Pcl_tutorial();
    ~Pcl_tutorial();

    void cylinder_segmentation(pcl::PointCloud<PointT>::Ptr cloud
                              ,pcl::PointCloud<PointT>::Ptr cloud_cylinder
                              ,pcl::PointCloud<PointT>::Ptr cloud_plane);

    void passthrough(pcl::PointCloud<PointTRGB>::Ptr cloud
                    ,pcl::PointCloud<PointTRGB>::Ptr cloud_filtered
                    ,char* direction,float coordinate_min, float coordinate_Max);

    void calculate_normal(pcl::PointCloud<PointTRGB>::Ptr point_cloud_ptr
                         ,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals1);

    void downsampling(pcl::PointCloud<PointTRGB>::Ptr cloud
                    , pcl::PointCloud<PointTRGB>::Ptr cloud_filtered
                    , float range);

};

#endif //PCL_TUTORIAL_H