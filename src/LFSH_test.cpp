#include <iostream>
#define PCL_NO_PRECOMPILE
#include "LFSH.hpp"

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/histogram_visualizer.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

int main(int argc, char** argv)
{

    std::cout << "Begin"<< std::endl;

    //Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        p_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZ>::Ptr
        p_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*p_src_ptr);

    //For test
    std::cout << p_src_ptr->size() << std::endl;

    pcl::LFSHEstimation<pcl::PointXYZ,pcl::PointNormal,pcl::LFSHSignature> lfsh_extract;

    //Add transform build a new point cloud
    lfsh_extract.setInputCloud(p_src_ptr);

    pcl::PointCloud<pcl::PointNormal>::Ptr
        p_targetnormal_ptr(new pcl::PointCloud<pcl::PointNormal>);
    {
//        pcl::ScopeTime t("Estimate normals for scene");
        // Estimate normals for scene
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> nest;
        nest.setRadiusSearch (0.006);
        nest.setInputCloud (p_src_ptr);
        nest.compute (*p_targetnormal_ptr);
    }
    pcl::PointCloud<LFSHSignature>::Ptr
        p_targetLFSH_ptr(new pcl::PointCloud<LFSHSignature>);
    lfsh_extract.compute(*p_targetLFSH_ptr);

    //Registration
    typedef pcl::KdTreeFLANN<LFSHSignature>::Ptr FeatureKdTreePtr;
    FeatureKdTreePtr m_feature_tree;
    m_feature_tree.reset(new pcl::KdTreeFLANN<LFSHSignature>);
    m_feature_tree->setInputCloud (p_targetLFSH_ptr);




    return 0;
}
