//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "0.Utils/PcTransformations.h"

void PcTransformations::transformPointCloudWithAngles(pcl::PointCloud<PointType>::Ptr cloud, float translation_x, float translation_y, float translation_z, float yaw, float pitch, float roll){
    if (cloud->empty()) {
    std::cerr << "Error: Point cloud is empty. Cannot perform transformation." << std::endl;
    return;
    }

    // Define the transformation matrix using Eigen
    Eigen::Affine3f transformation_matrix = Eigen::Affine3f::Identity();

    // Apply translation
    transformation_matrix.translation() << translation_x, translation_y, translation_z;

    // Apply rotation (yaw, pitch, roll)
    transformation_matrix.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    transformation_matrix.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transformation_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    // Apply the transformation to the point cloud
    pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);
}