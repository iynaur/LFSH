#include <iostream>

#include "LFSH.hpp"

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/histogram_visualizer.h>

#include <pcl/visualization/pcl_visualizer.h>



int main()
{

    std::cout << "Begin"<< std::endl;

    //Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        p_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZ>::Ptr
        p_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./a.pcd",*p_src_ptr);

    //For test
    std::cout << p_src_ptr->size() << std::endl;

    pcl::LFSHEstimation<pcl::PointXYZ,pcl::PointNormal,pcl::LFSHSignature30> lfsh_extract;

    //Add transform build a new point cloud
    //


    //Registration





    return 1;
}
