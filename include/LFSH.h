#ifndef PCL_FETURE_LFSH_H_
#define PCL_FETURE_LFSH_H_

#include <pcl/features/normal_3d_omp.h>

#include <pcl/features/feature.h>

#include "LFSHSignature30.h"

namespace pcl
{
    template<typename PointT,typename PointNT,typename PointOUTT=pcl::LFSHSignature30>
        class LFSHEstimation:public FeatureFromNormals<PointT,PointNT,PointOUTT>
    {
        public:
            typedef boost::shared_ptr<LFSHEstimation<PointT,PointNT,PointOUTT> > Ptr;
            typedef boost::shared_ptr<const LFSHEstimation<PointT,PointNT,PointOUTT> > ConstPtr;
            using Feature<PointT,PointOUTT>::feature_name_;
            using Feature<PointT,PointOUTT>::getClassName;
            using Feature<PointT,PointOUTT>::indices_;
            using Feature<PointT,PointOUTT>::k_;
            using Feature<PointT,PointOUTT>::search_parameter_;
            using Feature<PointT,PointOUTT>::input_;
            using Feature<PointT,PointOUTT>::surface_;
            using FeatureFromNormals<PointT,PointNT,PointOUTT>::normals_;

            typedef typename Feature<PointT,PointOUTT>::PointCloudOut PointCloudOut;

            /** \brief Empty constructor. */
            LFSHEstimation():
                nr_bins_f1_(10),nr_bins_f2_(10),nr_bins_f3_(10),
                hist_f1_(),hist_f2_(),hist_f3_(),lfsh_histogram_(),
                d_pi_(1.0f/(2.0f*static_cast<float>(M_PI)))
        {
            feature_name_ = "LFSHEstimation";
        };




    };
}

#endif
