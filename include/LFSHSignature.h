#pragma once   
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace pcl
{
	struct LFSHSignature
	{
		//N1 local depth:10	 0-9
		//N2 deviation angle:15	  10-24
		//N3 density :5	   25-29
		float histogram[30];

		static int getNumberOfDimensions() { return 30; };

		static int descriptorSize() { return 30; }

	};
 
}
  POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::LFSHSignature,
		(float[30],histogram,lfsh)
		)	
