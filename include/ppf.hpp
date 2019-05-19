#ifndef PCL_FEATURE_LFSH_IMPL_H_
#define PCL_FEATURE_LFSH_IMPL_H_

#include "ppf.h"




//#define LFSH_DEBUG

template<typename PointInT, typename PointNT, typename PointOutT>
void pcl::PPFEstimation<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut &output)
{

}

template<typename RealT>
void hist2(const typename std::vector<RealT>& data1,
        const typename std::vector<RealT>& data2,
        size_t bins1,
        size_t bins2,
        size_t* result,
        RealT lower1,
        RealT upper1,
        RealT lower2,
        RealT upper2) {
    // Sanity checks
    assert(data1.size() == data2.size());
    assert(bins1 > 0 && bins2 > 0);

    // Total number of bins
    const size_t bins = bins1 * bins2;

    // Initialize result to zero
    std::fill(result, result + bins, size_t(0));

    // Increments
    const RealT inc1 = RealT(bins1) / (upper1 - lower1);
    const RealT inc2 = RealT(bins2) / (upper2 - lower2);

    // Compute absolute 2D histogram
    for(size_t i = 0; i < data1.size(); ++i) {
        const size_t idx1 = std::max(size_t(0), std::min(bins1 - 1, size_t((data1[i] - lower1) * inc1)));
        const size_t idx2 = std::max(size_t(0), std::min(bins2 - 1, size_t((data2[i] - lower2) * inc2)));

        // Row-major storage: stride equal to second dimension
        ++result[idx1 * bins2 + idx2];
    }
}

template<typename RealT>
void rhist2(const typename std::vector<RealT>& data1,
            const typename std::vector<RealT>& data2,
            size_t bins1,
            size_t bins2,
            RealT* result,
            RealT lower1,
            RealT upper1,
            RealT lower2,
            RealT upper2) {
    // Number of bins
    const size_t bins = bins1 * bins2;

    // Special case
    if(data1.empty()) {
        std::fill(result, result + bins, RealT(0));
        return;
    }

    // Compute absolute histogram
    size_t histabs[bins];
    hist2(data1, data2, bins1, bins2, histabs, lower1, upper1, lower2, upper2);

    // Normalize
    for(size_t i = 0; i < bins; ++i)
        result[i] = RealT(histabs[i]) / RealT(data1.size());
}

template<typename PointInT, typename PointNT, typename PointOutT>
bool pcl::PPFEstimation<PointInT, PointNT, PointOutT>::compute(PointCloudOut& output)
{



    {
        input_ptr_ = input_;
        kdtree_ptr_.reset(new pcl::search::KdTree<PointInT>);
        kdtree_ptr_->setInputCloud(input_ptr_);

        boost::shared_ptr<pcl::search::KdTree<PointInT> > tmp_kd_tree(new pcl::search::KdTree<PointInT>);

        pcl::NormalEstimationOMP<PointInT, PointNT> ne;
        ne.setInputCloud(input_ptr_);
        ne.setSearchMethod(tmp_kd_tree);
        ne.setKSearch(8);
        //    ne.setNumberOfThreads(4);
        ne.compute(*normal_ptr_);
    }
    //TODO: 可以尝试实现用高密度点云估算法向量方向 setSearchSurface.

    std::vector<int> pointIdx_vector;
    std::vector<float> pointDistance_vector;

    //vector 点乘 x.dot(y)

    std::cout << "input size:" << input_ptr_->size() << std::endl;
    output.clear();
    output.resize(input_ptr_->size());

    /*
     * Main loop over all input points
     */
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for(size_t i = 0; i < input_ptr_->size(); ++i) {
        // Take current source point
        PointNT pi;
        pi.x = input_ptr_->points[i].x;
        pi.y = input_ptr_->points[i].y;
        pi.z = input_ptr_->points[i].z;
        pi.normal_x = input_ptr_->points[i].normal_x;
        pi.normal_y = input_ptr_->points[i].normal_y;
        pi.normal_z = input_ptr_->points[i].normal_z;

        std::fill(output.points[i].histogram, output.points[i].histogram + PPFHistDim, 0.0f);
        // Skip if source is non-finite in XYZ or normal
        if(!pcl::isFinite(pi) || !pcl_isfinite(pi.normal_x) || !pcl_isfinite(pi.normal_y) || !pcl_isfinite(pi.normal_z)) {

            continue;
        }

        // Find neighbors
        if (kdtree_ptr_->radiusSearch(input_ptr_->at(i), r_, pointIdx_vector, pointDistance_vector) <= 0)
        {
            continue;
        };



        // Relations for 2D histogramming
        std::vector<float> delta;
        std::vector<float> gamma;

        // Loop over neighbors and compute relations
        for(size_t j = 0; j < pointIdx_vector.size(); ++j) {
            // Take neighbor
            const PointNT& pj = input_ptr_->points[pointIdx_vector[j]];

            // Skip neighbor if it does not have a valid normal
            if(!pcl_isfinite(pj.normal_x) || !pcl_isfinite(pj.normal_y) || !pcl_isfinite(pj.normal_z))
                continue;



            // Compute point distance (delta), skip if too small
            const float deltaij = sqrtf(pointDistance_vector[j]);
            if(deltaij < 1e-5f)
                continue;

            // Compute normalized direction vector used below
            const float dx = (pj.x - pi.x) / deltaij;
            const float dy = (pj.y - pi.y) / deltaij;
            const float dz = (pj.z - pi.z) / deltaij;

            // Compute angle cosine between direction vector and neighbor normal
            const float gammaij = pj.normal_x * dx + pj.normal_y * dy + pj.normal_z * dz;
//                    const float gammaij = pj.normal_x * pi.normal_x + pj.normal_y * pi.normal_y + pj.normal_z * pi.normal_z; // alpha
//                    const float gammaij = dx * pi.normal_x + dy * pi.normal_y + dz * pi.normal_z; // beta


            // Store relations
            delta.push_back(deltaij);
            gamma.push_back(gammaij);
        } // End loop over neighbors (j)

//                // Put into histogram
        rhist2<float>(delta, gamma, NDist, NAngle, output.points[i].histogram, 0.0f, r_, -1.0f, 1.0f);
//                core::rhist2<float>(delta, gamma, NDist, NAngle, output.points[i].histogram, 0.0f, _radius, 0.0f, 1.0f);
//                core::rhist2<float>(delta, gamma, NDist, NAngle, output.points[i].histogram, 0.0f, _radius, -1.0f, 1.0f);
    } // End main loop over all input points (i)

#ifdef LFSH_DEBUG
    std::cout << "1:" << index_1_min << ":" << index_1_max << std::endl;
    std::cout << "2:" << index_2_min << ":" << index_2_max << std::endl;
    std::cout << "3:" << index_3_min << ":" << index_3_max << std::endl;
#endif
    return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
float pcl::PPFEstimation<PointInT, PointNT, PointOutT>::dot(Eigen::Vector3f x, Eigen::Vector3f y)
{
    return x(0) * y(0) + x(1) * y(1) + x(2) * y(2);
}



template<typename PointInT, typename PointNT, typename PointOutT>
int pcl::PPFEstimation<PointInT, PointNT, PointOutT>::computeDeviationAngle(
    Eigen::Vector3f source_normal,
    Eigen::Vector3f target_normal) const
{
    float theta(0.0);
    if (dot(target_normal, source_normal) > 0.995)
    {
        theta = 0.0;
    }
    else
    {
        theta = acos(/*fabs*/dot(target_normal, source_normal));
    }
    if (theta < 0 || theta > M_PI)
        theta = M_PI;
    //std::cout << "m_pi" << M_PI << std::endl;
    if (theta < 0.00001) return 0;//TODO:先对付着，要找一下异常值得原因
//    std::cerr<<theta<<std::endl;
    int res;
    return res;
}



#endif //PCL_FEATURE_LFSH_H_
