#include <iostream>

#include "LFSH.h"

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/histogram_visualizer.h>

#include <pcl/visualization/pcl_visualizer.h>




int main()
{

    std::cout << "Begin"<< std::endl;

    //Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        p_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("./a.pcd",*p_src_ptr);

    //For test
    std::cout << p_src_ptr->size() << std::endl;

    pcl::LFSHEstimation lfsh_extract;






    return 1;
}
