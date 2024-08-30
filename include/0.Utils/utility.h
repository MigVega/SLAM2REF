//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#ifndef _UTILITY_
#define _UTILITY_
#pragma once

//#include <ros/ros.h>
//
//#include <std_msgs/Header.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/NavSatFix.h>
//#include <sensor_msgs/image_encodings.h>
//
//#include <nav_msgs/Odometry.h>
//#include <nav_msgs/Path.h>
//
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
//#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
//#include <pcl_conversions/pcl_conversions.h>

//#include <pcl/filters/passthrough.h>

//#include <tf/LinearMath/Quaternion.h>
//#include <tf/transform_listener.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>
 
#include <opencv2/highgui/highgui.hpp>
//#include <image_transport/image_transport.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <set>
#include <map>
#include <algorithm>
#include <utility>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <boost/filesystem.hpp> //for copy function
#include <filesystem> // requires gcc version >= 8

// OPEN 3D //////////////////////////////
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <iostream>
#include <memory>

#include "0.Utils/Timer.h"

namespace fs = std::filesystem;
using std::ios;
using std::cout;
using std::cerr;
using std::endl;

// struct PointXYZIS
// {
//     PCL_ADD_POINT4D
//     float intensity;
//     float score;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIS,  
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, score, score)
// )

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))
using PointTypePose = PointXYZIRPYT;
//pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
//using PointType = pcl::PointXYZI;
using PointType = pcl::PointXYZ; // MV: changed since intensity is not used here
using PointTypeRGB = pcl::PointXYZRGB;

struct G2oLineInfo 
{
    std::string type;

    int prev_idx = -1; // for vertex, this member is null
    int curr_idx;

    std::vector<double> trans;
    std::vector<double> quat;

    inline static const std::string kVertexTypeName = "VERTEX_SE3:QUAT";
    inline static const std::string kEdgeTypeName = "EDGE_SE3:QUAT";
}; // G2oLine


struct SphericalPoint
{
    float az; // azimuth 
    float el; // elevation
    float r; // radius
};

std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr);



namespace Slam3refParam {

    /**
     * @brief MV: this a space/memory reservation for the nodes of each session on the graph.
     * The poses of the first session will be added form the index 0 in the graph, 
     * and the ones of the session 1 will be added from the index Nr. kSessionStartIdxOffset (now 1.000.000).
     * Therefore, there can not be a session with more than this number of poses!! [WARNING]
    */
    const static inline int kSessionStartIdxOffset = 1000000; // int max 2147483647

    int genGlobalNodeIdx (const int& _session_idx, const int& _node_offset);
    int genAnchorNodeIdx (const int& _session_idx);
} // namespace Slam3refParam



pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);
pcl::PointCloud<PointTypeRGB>::Ptr transformPointCloud(pcl::PointCloud<PointTypeRGB>::Ptr cloudIn, PointTypePose* transformIn);

float rad2deg(float radians); 
float deg2rad(float degrees);
void fsmkdir(std::string _path);

void readBin(const std::string& _bin_path, const pcl::PointCloud<PointType>::Ptr& _pcd_ptr);

//sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

std::vector<double> splitPoseLine(const std::string& _str_line, char _delimiter);

/**
 * @brief Parses a line from a G2O file and extracts information to create a G2oLineInfo structure.
 *
 * This function takes a string representing a line from a G2O file and parses it to extract relevant information.
 * The parsed information is then used to create a G2oLineInfo structure, which contains details such as the line type (edge or node),
 * indices, translation, and quaternion.
 *
 * @param _str_line The string representing a line from the G2O file.
 * @return A G2oLineInfo structure with parsed information.
 *
 * @see G2oLineInfo, isTwoStringSame
 *
 * @code
 * // Example Usage:
 * std::string g2oLine = "VERTEX_SE3:QUAT 0 1 2 3 4 5 6";
 * G2oLineInfo parsedInfo = splitG2oFileLine(g2oLine);
 * @endcode
 */
G2oLineInfo splitG2oFileLine(const std::string& _str_line);

SphericalPoint cart2sph(const PointType & _cp);

std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio);



template<typename T>
cv::Mat convertColorMappedImg (const cv::Mat &_src, std::pair<T, T> _caxis)
{
  T min_color_val = _caxis.first;
  T max_color_val = _caxis.second;

  cv::Mat image_dst;
  image_dst = 255 * (_src - min_color_val) / (max_color_val - min_color_val);
  image_dst.convertTo(image_dst, CV_8UC1);
  
  cv::applyColorMap(image_dst, image_dst, cv::COLORMAP_JET);

  return image_dst;
}

std::set<int> convertIntVecToSet(const std::vector<int> & v);

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

//sensor_msgs::ImagePtr cvmat2msg(const cv::Mat &_img);

//void pubRangeImg(cv::Mat& _rimg, sensor_msgs::ImagePtr& _msg, image_transport::Publisher& _publiser, std::pair<float, float> _caxis);
//void publishPointcloud2FromPCLptr(const ros::Publisher& _scan_publisher, const pcl::PointCloud<PointType>::Ptr _scan);
//sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

//template<typename T>
//double ROS_TIME(T msg)
//{
//    return msg->header.stamp.toSec();
//}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
Eigen::Affine3f pclPointToAffine3fOnlyTranslation(PointTypePose thisPoint);

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

float pointDistance(PointType p);
float pointDistance(PointType p1, PointType p2);

// Test if two strings are exactly the same
bool isTwoStringSame(const std::string& _str1, std::string _str2);

/**
 * @brief Collects individual digits from a given number and stores them in a vector.
 *
 * This recursive function extracts individual digits from the input number and appends them to the provided vector.
 *
 * @param[out] digits A vector to store the individual digits of the input number.
 * @param num The number from which digits are to be extracted.
 *
 * @note The function uses recursion to handle multi-digit numbers, extracting digits from the least significant to the most significant.
 * @note The collected digits are stored in the vector in reverse order, i.e., from the most significant to the least significant.
 */
void collect_digits(std::vector<int>& digits, int num);

// save the trajectory in the kitti dataset pose format: https://github.com/MichaelGrupp/evo/wiki/Formats#kitti---kitti-dataset-pose-format
void writePose3ToStream(std::fstream& _stream, const gtsam::Pose3& _pose);
void writePose3ToStream_CC(std::fstream& _stream, const gtsam::Pose3& _pose);
void writePose3ToStream_CC_withColor(std::fstream& _stream, const gtsam::Pose3& _pose, std::vector<int> color);

void writeAllPosesWithColorsForCC(std::fstream& _stream, const std::vector<std::pair<int, gtsam::Pose3>>& poseVector, const std::vector<std::pair<int, int>>& poseCategoryVector);
std::vector<int> linspace(int a, int b, int N);

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter);
Eigen::MatrixXd readSCD(std::string fileToOpen);

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2);



/**
 * @brief Transforms a point cloud of type pcl::PointXYZ to pcl::PointXYZRGB and assigns all points the specified color.
 *
 * This is a utility function for coloring a point cloud. The color is provided as an RGB value. Each channel (Red, Green, Blue)
 * has a value in the range [0, 255].
 *
 * @param cloud Input point cloud (type pcl::PointXYZ).
 * @param color RGB color value to be assigned (type Eigen::Vector3i). e.g.   Eigen::Vector3i color(255, 0, 0); // This represents red color
 *
 * @return Point cloud with all points colored according to the input RGB value (type pcl::PointXYZRGB).
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector3i& color);
void colorOpen3DPointCloud(open3d::geometry::PointCloud& open3d_cloud, const Eigen::Vector3i& color);

pcl::PointCloud<pcl::PointXYZ>::Ptr radiusFilterPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        double minRadius,
        double maxRadius);


/**
 * @brief Projects the point cloud on a 2D grid in the XY plane and filters out points that are not part of a dense cell.
 *
 * This function takes in a point cloud and the size of a grid. It projects the point cloud onto a 2D grid in the XY plane.
 * Each cell in the grid contains the points that fall inside it. Then the grid cells with a number of points less than
 * the specified threshold are filtered out, i.e. the points in those cells are not included in the resulting point cloud.
 * The function returns a new point cloud which only includes points from grid cells that are sufficiently dense.
 *
 * @param inputCloud Pointer to the input point cloud to be filtered.
 * @param gridSize Size of the grid cells.
 * @param minPointsInGrid Minimum number of points required in a grid cell for it to be included in the output.
 * @return pcl::PointCloud<PointType>::Ptr Pointer to the newly created point cloud after filtering.
 */
pcl::PointCloud<PointType>::Ptr filterPointsWithGrid(
        pcl::PointCloud<PointType>::Ptr inputCloud,
        const double gridSize,
        const size_t minPointsInGrid);


void rotatePointCloud(pcl::PointCloud<PointType>::Ptr cloud, float yaw_angle);


void pclToOpen3d(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud);
std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3d_v2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);

void open3dToPcl(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);


std::vector<int> findKNearestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudKeyPoses6D,
                                    const pcl::PointXYZ& inputQueryPoint,
                                    int k);

// transformation from PointTypePose to pcl::PointXYZ
pcl::PointXYZ transformPoint(const PointTypePose& pose);

// convert PointCloud<PointTypePose> to PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud(const pcl::PointCloud<PointTypePose>::Ptr& inputCloud);


void loadPointCloud(const std::string &filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

void downsamplePointCloud(pcl::PointCloud<PointType>::Ptr& originalCloud, float leaf_size);

pcl::PointCloud<PointType>::Ptr cropPointCloudWithinSphere(
        const pcl::PointCloud<PointType>::Ptr& target_cloud,
        const Eigen::Vector3f& center,
        const double sphere_radius, const int numberOfCores
);
/// END OF PC utils

// Function that gets pose transformation
gtsam::Pose3 getPoseTransformation(const Eigen::Matrix4d& resultMatrix);



// File utilities
void copyFile(std::string sourcePath, std::string destinationDir);


// Custom logging function using printf-style formatting
void logInfo_print(const char* format, ...) ;


// Template logging function that accepts multiple arguments
template<typename... Args>
void logInfo_cout(Args... args) {
    std::ostringstream oss;
    // Expanding the arguments into the stream
    (oss << ... << args);
    // Outputting with color formatting
    std::cout << oss.str() << std::endl;
}

#endif //_UTILITY_