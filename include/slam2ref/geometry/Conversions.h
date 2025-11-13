//
// Created by refactoring utility.h 2024
// Format conversion functions between PCL, GTSAM, Open3D, and Eigen
//
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <memory>

// Point type definitions
typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGB PointTypeRGB;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
    (double, time, time)
)

typedef PointXYZIRPYT PointTypePose;

// PCL to GTSAM conversions
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
Eigen::Affine3f pclPointToAffine3fOnlyTranslation(PointTypePose thisPoint);
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);
gtsam::Pose3 getPoseTransformation(const Eigen::Matrix4d& resultMatrix);

// PCL to Open3D conversions
void pclToOpen3d(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                 std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud);
std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3d_v2(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
void open3dToPcl(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);

// Point cloud type conversions
pcl::PointXYZ transformPoint(const PointTypePose& pose);
pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud(
    const pcl::PointCloud<PointTypePose>::Ptr& inputCloud);
