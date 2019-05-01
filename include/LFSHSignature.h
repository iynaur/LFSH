#pragma once
//#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#define N1  13
#define N2   13
#define N3   1

#define TOTAL_N N1+N2+N3

namespace pcl
{
typedef  FPFHSignature33 LFSHSignature;
//  struct LFSHSignature
//  {

//    float histogram[TOTAL_N];

//    static int getNumberOfDimensions() { return TOTAL_N; };

//    static int descriptorSize() { return TOTAL_N; }

//  };

}
//  POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::LFSHSignature,
//    (float[TOTAL_N],histogram,lfsh)
//    )
