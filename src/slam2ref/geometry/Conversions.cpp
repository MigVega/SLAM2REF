//
// Created by refactoring utility.cpp 2024
// Format conversion functions implementation
//

#include "slam2ref/geometry/Conversions.h"
#include <pcl/common/transforms.h>

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                                   thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f pclPointToAffine3fOnlyTranslation(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, 0, 0, 0);
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

gtsam::Pose3 getPoseTransformation(const Eigen::Matrix4d& resultMatrix)
{
    gtsam::Rot3 rotation(resultMatrix.block<3,3>(0, 0));
    gtsam::Point3 translation(resultMatrix(0, 3), resultMatrix(1, 3), resultMatrix(2, 3));
    return gtsam::Pose3(rotation, translation);
}

void pclToOpen3d(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                 std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud)
{
    open3d_cloud->points_.clear();
    open3d_cloud->points_.reserve(pcl_cloud->size());

    for (const auto& point : pcl_cloud->points) {
        open3d_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
}

std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3d_v2(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
{
    auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();
    open3d_cloud->points_.reserve(pcl_cloud->size());

    for (const auto& point : pcl_cloud->points) {
        open3d_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    return open3d_cloud;
}

void open3dToPcl(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud)
{
    pcl_cloud->clear();
    pcl_cloud->reserve(open3d_cloud->points_.size());

    for (const auto& point : open3d_cloud->points_) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(point.x());
        pcl_point.y = static_cast<float>(point.y());
        pcl_point.z = static_cast<float>(point.z());
        pcl_cloud->push_back(pcl_point);
    }
}

pcl::PointXYZ transformPoint(const PointTypePose& pose)
{
    pcl::PointXYZ point;
    point.x = pose.x;
    point.y = pose.y;
    point.z = pose.z;
    return point;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud(
    const pcl::PointCloud<PointTypePose>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
    outputCloud->reserve(inputCloud->size());

    for (const auto& point : inputCloud->points) {
        outputCloud->push_back(transformPoint(point));
    }

    return outputCloud;
}
