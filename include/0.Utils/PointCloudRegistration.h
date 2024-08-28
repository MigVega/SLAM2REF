//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//


#ifndef slam2ref_PC_REGISTRATION_H
#define slam2ref_PC_REGISTRATION_H

#include "0.Utils/utility.h"


class PointCloudRegistration
{
public:
    PointCloudRegistration(void) {};
    ~PointCloudRegistration(void) {};

    static std::tuple<Eigen::Matrix4d, double, double> make_icp_v5_open3D(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Eigen::Matrix4d &initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration);

    static std::tuple<Eigen::Matrix4d, double, double> make_icp_v6_only_yaw(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Eigen::Matrix4d& initial_guess, int iterations, double thr_max_dist,bool visual, bool print_after_each_iteration);

    static std::tuple<Eigen::Matrix4d, double, double> make_icp_v7(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Eigen::Matrix4d &initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration, const double voxel_size, double radius, int max_nn);

    static std::tuple<Eigen::Matrix4d, double, double> make_icp_v8_p2p(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Eigen::Matrix4d &initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration, bool downsampleSource=true);

    static std::tuple<Eigen::Matrix4d, double> PerformYawGICP(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_queryKeyframeCloud, const std::shared_ptr<open3d::geometry::PointCloud>& open3d_centralKeyframeCloud, const double thr_1, const double thr_2, const int version);


    static std::pair<double, double>  EvaluateRegistration(const open3d::geometry::PointCloud &source,
                                const open3d::geometry::PointCloud &target,
                                double max_correspondence_distance,
                                const Eigen::Matrix4d &result_transformation, float range_distance);


    static std::pair<double, double> EvaluateRegistrationAlignedSource(open3d::geometry::PointCloud &alignedSource,
                                      const open3d::geometry::PointCloud &target,
                                      double max_correspondence_distance, float range_distance);

    static std::tuple<double, double, double, double> EvaluateRegistrationAlignedSourceAndRMSE(
            open3d::geometry::PointCloud& alignedSource,
            const open3d::geometry::PointCloud& target,
            double max_correspondence_distance,
            float range_distance);
};



#endif //slam2ref_PC_REGISTRATION_H