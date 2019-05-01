#ifndef PCL_FEATURE_LFSH_IMPL_H_
#define PCL_FEATURE_LFSH_IMPL_H_

#include "LFSH.h"




//#define LFSH_DEBUG

template<typename PointInT, typename PointNT, typename PointOutT>
void pcl::LFSHEstimation<PointInT, PointNT, PointOutT>::computeFeature(PointCloudOut &output)
{

}

template<typename PointInT, typename PointNT, typename PointOutT>
bool pcl::LFSHEstimation<PointInT, PointNT, PointOutT>::compute(PointCloudOut& output)
{
    output.clear();
    //For debug:
#ifdef LFSH_DEBUG
    const int index_1_max(29), index_1_min(0);
    const int index_2_max(29), index_2_min(0);
    const int index_3_max(29), index_3_min(0);
#endif
    //TODO: 这里有个问题。//解决了，注意初始化指针。

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
    //TODO: 可以尝试实现用高密度点云估算法向量方向 setSearchSurface.

    std::vector<int> pointIdx_vector;
    std::vector<float> pointDistance_vector;

    //vector 点乘 x.dot(y)

    std::cout << "input size:" << input_ptr_->size() << std::endl;
    int i(0);

    for (i = 0; i < (input_ptr_->size()); ++i)
    {
        //std::cout << "i:" << i << std::endl;

        if (kdtree_ptr_->radiusSearch(input_ptr_->at(i), r_, pointIdx_vector, pointDistance_vector) > 0)
        {
            pcl::LFSHSignature tmp_signature{};


            Eigen::Vector3f the_point_f;
            the_point_f(0) = input_ptr_->at(i).x;
            the_point_f(1) = input_ptr_->at(i).y;
            the_point_f(2) = input_ptr_->at(i).z;

            Eigen::Vector3f tmp_point;//q_i^j in paper
            Eigen::Vector3f tmp_source_normal;//n_i in paper
            Eigen::Vector3f tmp_target_normal;//n_i^j in paper

            tmp_source_normal = Eigen::Vector3f(
                normal_ptr_->at(i).normal_x,
                normal_ptr_->at(i).normal_y,
                normal_ptr_->at(i).normal_z);


            for (int j(0); j < pointIdx_vector.size(); ++j)
            {
                tmp_point(0) = input_ptr_->at(pointIdx_vector.at(j)).x;
                tmp_point(1) = input_ptr_->at(pointIdx_vector.at(j)).y;
                tmp_point(2) = input_ptr_->at(pointIdx_vector.at(j)).z;

                tmp_target_normal = Eigen::Vector3f(
                    normal_ptr_->at(pointIdx_vector[j]).normal_x,
                    normal_ptr_->at(pointIdx_vector[j]).normal_y,
                    normal_ptr_->at(pointIdx_vector[j]).normal_z);

                int tmp_index_1(0);
                int tmp_index_2(0);
                int tmp_index_3(0);

                tmp_index_1 = computeLocalDepth(tmp_source_normal, the_point_f, tmp_point);
#ifdef LFSH_DEBUG
                if (tmp_index_1 > index_1_max) tmp_index_1 = index_1_max;
                if (tmp_index_1 < index_1_min) tmp_index_1 = index_1_min;
#endif
                tmp_signature.histogram[tmp_index_1] += (1.0f);

                tmp_index_2 = N1_ + computeDeviationAngle(tmp_source_normal, tmp_target_normal);
#ifdef LFSH_DEBUG
                if (tmp_index_2 > index_2_max) tmp_index_2 = index_2_max;
                if (tmp_index_2 < index_2_min) tmp_index_2 = index_2_min;
#endif
                tmp_signature.histogram[tmp_index_2] += (1.0f);


                tmp_index_3 = N1_ + N2_ + computeDensity(tmp_source_normal, the_point_f, tmp_point);
#ifdef LFSH_DEBUG
                if (tmp_index_3 > index_3_max) tmp_index_3 = index_3_max;
                if (tmp_index_3 < index_3_min) tmp_index_3 = index_3_min;
#endif
                tmp_signature.histogram[tmp_index_3] += (1.0f);
            }
            /****************************************************/
            double test_sum(0.0);

            for (int k(0); k < TOTAL_N; ++k)
            {
                tmp_signature.histogram[k] = tmp_signature.histogram[k] / double(pointIdx_vector.size());
                test_sum += tmp_signature.histogram[k];
                //std::cout << ":" << tmp_signature.histogram[k];
            }
            //std::cout << std::endl;
            //std::cout << "test_sum:" << test_sum<<std::endl;
            //TODO:注意下面断言被注释掉了。
            //assert(test_sum != 3 && "Some error in compute local point feature histogram.");
            output.push_back(tmp_signature);
        }
        else
        {
            std::cout << "error:" << kdtree_ptr_->radiusSearch(input_ptr_->at(i), r_, pointIdx_vector, pointDistance_vector)
                << std::endl;
        }
    }
#ifdef LFSH_DEBUG
    std::cout << "1:" << index_1_min << ":" << index_1_max << std::endl;
    std::cout << "2:" << index_2_min << ":" << index_2_max << std::endl;
    std::cout << "3:" << index_3_min << ":" << index_3_max << std::endl;
#endif
    return true;
}

template<typename PointInT, typename PointNT, typename PointOutT>
float pcl::LFSHEstimation<PointInT, PointNT, PointOutT>::dot(Eigen::Vector3f x, Eigen::Vector3f y)
{
    return x(0) * y(0) + x(1) * y(1) + x(2) * y(2);
}

template<typename PointInT, typename PointNT, typename PointOutT>
int pcl::LFSHEstimation<PointInT, PointNT, PointOutT>::computeLocalDepth(
    Eigen::Vector3f source_normal,
    Eigen::Vector3f source_point,
    Eigen::Vector3f target_point) const
{
    double d(r_);
    d -= (dot(source_normal, target_point - source_point));
    int res = int(d / 2 / r_ * N1_);
    assert(res>=0 && res< N1_);
    return res;
}

template<typename PointInT, typename PointNT, typename PointOutT>
int pcl::LFSHEstimation<PointInT, PointNT, PointOutT>::computeDeviationAngle(
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
    int res = int(theta / M_PI * N2_);
    assert(res >= 0 && res < N2_);
    return res;
}

template<typename PointInT, typename PointNT, typename PointOutT>
int pcl::LFSHEstimation<PointInT, PointNT, PointOutT>::computeDensity(
    Eigen::Vector3f source_normal,
    Eigen::Vector3f source_point,
    Eigen::Vector3f target_point) const
{
    double d(0.0);
    source_point = source_point - target_point;
    d = sqrt((source_point(0) * source_point(0) + source_point(1) * source_point(1) + source_point(2) * source_point(2))
        - pow(dot(source_point, source_normal), 2));
    int res = int(d / r_ * N3_);
    assert( res >=0 && res < N3_);
    return res;
}

#endif //PCL_FEATURE_LFSH_H_
