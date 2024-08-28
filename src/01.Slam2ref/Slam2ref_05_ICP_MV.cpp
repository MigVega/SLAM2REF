//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include <utility>

#include "01.Slam2ref/Slam2ref.h"
#include "0.Utils/PointCloudRegistration.h"
#include "0.Utils/PcTransformations.h"
#include <numeric>

void translatePointCloud(pcl::PointCloud<PointType>::Ptr cloud, float translation_x, float translation_y, float translation_z) {
    if (cloud->empty()) {
        std::cerr << "Error: Point cloud is empty. Cannot perform translation." << std::endl;
        return;
    }

    // Define the translation vector using Eigen
    Eigen::Affine3f translation_matrix = Eigen::Affine3f::Identity();
    translation_matrix.translation() << translation_x, translation_y, translation_z;

    // Apply the translation to the point cloud
    pcl::transformPointCloud(*cloud, *cloud, translation_matrix);
}




/* MV: Opposite to the previous function, this one takes the accumulated scans in the central session and transforms them to the local ref system,
 so that I can do a valid ICP with them (later the yaw angle  [calculated with SC] will be added)
  *
 */
std::optional<gtsam::Pose3> Slam2ref::doICPVirtualRelative_with_BIM_pc_v1( // For RS loop
        Session &target_sess, Session &source_sess,
        const int &loop_idx_target_session, const int &loop_idx_source_session, const size_t &index)
{
    // Part 1: Parse point clouds and initialize keyframe point cloud containers
    mtx.lock();

    pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());

    // Part 2: Retrieve keyframes in local coordinates from both central and source sessions
    // MV: the goal here is to retrieve a larger point cloud for the central, so that the source can match better with the map.
    source_sess.loopFindNearKeyframesGeneral(queryKeyframeCloud, loop_idx_source_session, 0,"local");
    target_sess.loopFindNearKeyframesGeneral(centralKeyframeCloud, loop_idx_target_session, NNKeyframes_for_submap_, "central");

    // Convert the index to a string and pad with zeros
    std::string paddedIndex = std::to_string(index + 1);
    paddedIndex = std::string(3 - paddedIndex.length(), '0') + paddedIndex;

    std::string dir = save_directory_ +"ICP_after_SC/";
    std::filesystem::create_directories(dir);

    // Color the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_centralKeyframeCloud = colorPointCloud(centralKeyframeCloud, green_color_utils);
    pcl::io::savePCDFileBinary(dir + paddedIndex +"_central_cloud.pcd", *colored_centralKeyframeCloud);


    // MV: 2.2 Translate the source point cloud to the position of the central point cloud in global coordinate system -> to be able to perform ICP with a well-constructed point cloud -> not as done in VirtualICP after SC loop detection
    float translation_x = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].x;
    float translation_y = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].y;
    float translation_z = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].z;


    float r_yaw = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].yaw;
    float r_pitch = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].pitch;
    float r_roll = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].roll;
    PcTransformations::transformPointCloudWithAngles(centralKeyframeCloud, -translation_x, -translation_y, -translation_z, -r_yaw, -r_pitch, -r_roll);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_centralKeyframeCloud_t = colorPointCloud(centralKeyframeCloud, green_color_utils);
    pcl::io::savePCDFileBinary(dir +  paddedIndex +"_central_cloud_2_transformed_to_local.pcd", *colored_centralKeyframeCloud_t);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_queryKeyframeCloud = colorPointCloud(queryKeyframeCloud, red_color_utils);
    pcl::io::savePCDFileBinary(dir +  paddedIndex +"_source_cloud.pcd", *colored_queryKeyframeCloud);


    mtx.unlock();

    // Part 3: ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(10); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter // MV: changed from 150 to 10 -> for indoor
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Part 4: Align point clouds using ICP
    icp.setInputSource(queryKeyframeCloud);
    icp.setInputTarget(centralKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);



    // Part 5: Check ICP convergence and fitness score
    if (!icp.hasConverged() || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        if(print_SC_ICP_test_values_for_each_iteration_)
        {
            mtx.lock();
            std::cout << "  [BAD  - SC loop Nr."<< paddedIndex << "] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
            mtx.unlock();
        }
        return std::nullopt;
    }
    else
    {

        mtx.lock();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_unused_result = colorPointCloud(unused_result, blue_color_utils);
        pcl::io::savePCDFileBinary(dir + paddedIndex +"_source_cloud_after_MV_transformation_afterICP.pcd", *colored_unused_result);

        if(print_SC_ICP_test_values_for_each_iteration_) {
            std::cout << "\033[32m  [GOOD - SC loop Nr." << paddedIndex << "] ICP fitness test passed ("
                      << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop. \033[0m"
                      << std::endl;
        }
        mtx.unlock();
    }

    // Part 6: Get pose transformation and return the relative pose
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;

    correctionLidarFrame = icp.getFinalTransformation();


    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    auto result = poseTo.between(poseFrom);

    return result;
} // doICPGlobalRelative




/* MV: Opposite to the previous version (v1), this one takes the accumulated scans in the central session and transforms them to the local ref system, so that I can do a valid ICP with them (later the yaw angle [calculated with SC] will be added)
  *
 */
std::optional<gtsam::Pose3> Slam2ref::doICPVirtualRelative_with_BIM_pc_v2_Yaw_XY_porjection(Session &target_sess,
                                                                                            Session &source_sess,
                                                                                            const int &loop_idx_target_session, const int &loop_idx_source_session, const float &yaw_angle_source, const size_t &index)
{
    // Part 1: Parse point clouds and initialize keyframe point cloud containers
    mtx.lock();

    pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());

    // Part 2: Retrieve keyframes in local coordinates from both central and source sessions
    // MV: the goal here is to retrieve a larger point cloud for the central, so that the source can match better with the map.
    // MV: the problem I detected while saving the central point cloud is that this PC is very distorted, because the frames are placed together but left in the local coordinate system, what actually does not make much sense
    source_sess.loopFindNearKeyframesGeneral(queryKeyframeCloud, loop_idx_source_session, 0,"local");
    target_sess.loopFindNearKeyframesGeneral(centralKeyframeCloud, loop_idx_target_session, NNKeyframes_for_submap_, "central"); // MV: originally historyKeyframeSearchNum = 25

    // Convert the index to a string and pad with zeros
    std::string paddedIndex = std::to_string(index + 1);
    paddedIndex = std::string(3 - paddedIndex.length(), '0') + paddedIndex;

    std::string dir = save_directory_ +"ICP_after_SC/";
    std::filesystem::create_directories(dir);

    // Color the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_centralKeyframeCloud = colorPointCloud(centralKeyframeCloud, green_color_utils);
    pcl::io::savePCDFileBinary(dir + paddedIndex +"_central_cloud.pcd", *colored_centralKeyframeCloud);
//    pcl::io::savePCDFileBinary("/home/mlegion/ws/09_bim_slam_mod_ltMapper/src/Slam2ref/data/source_cloud_"+ std::to_string(index+1) +"_bfr.pcd", *queryKeyframeCloud);
    // MV 2.1 Rotate the source point cloud with teh yaw angle
    rotatePointCloud(queryKeyframeCloud, yaw_angle_source);

    // MV: 2.2 Translate the source point cloud to the position of the central point cloud in global coordinate system -> to be able to perform ICP with a well constructed point cloud -> not as done in VirtualICP after SC loop detection
    float translation_x = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].x;
    float translation_y = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].y;
    float translation_z = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].z;
//    translatePointCloud(queryKeyframeCloud, translation_x, translation_y, translation_z);
    float r_yaw = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].yaw;
    float r_pitch = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].pitch;
    float r_roll = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].roll;
    PcTransformations::transformPointCloudWithAngles(centralKeyframeCloud, -translation_x, -translation_y, -translation_z, -r_yaw, -r_pitch, -r_roll); // MV: here I am moving the central scan (with 5 scans from the simulated scans in BIM) from the global to the local coord system -> where the central scan is at 0,0,0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_centralKeyframeCloud_t = colorPointCloud(centralKeyframeCloud, green_color_utils);
    pcl::io::savePCDFileBinary(dir +  paddedIndex +"_central_cloud_2_transformed_to_local.pcd", *colored_centralKeyframeCloud_t);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_queryKeyframeCloud = colorPointCloud(queryKeyframeCloud, red_color_utils);
    pcl::io::savePCDFileBinary(dir +  paddedIndex +"_source_cloud.pcd", *colored_queryKeyframeCloud);


    mtx.unlock();
    // Part 3: ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(10); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter // MV: changed from 150 to 10 -> for indoor
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Part 4: Align point clouds using ICP
    icp.setInputSource(queryKeyframeCloud);
    icp.setInputTarget(centralKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);



    // Part 5: Check ICP convergence and fitness score
    if (!icp.hasConverged() || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        if(print_SC_ICP_test_values_for_each_iteration_) {
            mtx.lock();
            std::cout << "  [BAD  - SC loop Nr." << paddedIndex << "] ICP fitness test failed ("
                      << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop."
                      << std::endl;
            mtx.unlock();
        }
        return std::nullopt;
    }
    else
    {

        mtx.lock();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_unused_result = colorPointCloud(unused_result, blue_color_utils);
        pcl::io::savePCDFileBinary(dir + paddedIndex +"_source_cloud_after_MV_transformation_afterICP.pcd", *colored_unused_result);
        if(print_SC_ICP_test_values_for_each_iteration_) {
            std::cout << "\033[32m  [GOOD - SC loop Nr." << paddedIndex << "] ICP fitness test passed ("
                      << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop. \033[0m"
                      << std::endl;
        }

        mtx.unlock();
    }

    // Part 6: Get pose transformation and return the relative pose
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;

    correctionLidarFrame = icp.getFinalTransformation();


    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    auto result = poseTo.between(poseFrom);

    return result;
} // doICPGlobalRelative




// MV TODO pass some of the next functions to utils or to new classes as static methods

void SaveCloudsToFile(const std::string& save_directory_, const std::string& paddedIndex, const std::shared_ptr<open3d::geometry::PointCloud>& aligned, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_centralKeyframeCloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_queryKeyframeCloud) {
    std::string dir = save_directory_ + "ICP_after_SC/";
    std::filesystem::create_directories(dir);
    open3d::io::WritePointCloud(dir + paddedIndex + "_source_cloud_after_MV_transformation_afterICP.pcd", *aligned);
    pcl::io::savePCDFileBinary(dir + paddedIndex + "_central_cloud_2_transformed_to_local.pcd", *colored_centralKeyframeCloud);
    pcl::io::savePCDFileBinary(dir + paddedIndex + "_source_cloud.pcd", *colored_queryKeyframeCloud);
}



void LogICPFitnessTestStatus_v2(bool print_SC_ICP_test_values_for_each_iteration_, double loopFitnessScoreThreshold_Open3D_, const bool& passed, const std::string& paddedIndex, double fitness) {
    if(print_SC_ICP_test_values_for_each_iteration_) {
        if (passed) {
            std::cout << "\033[32m [GOOD  - SC loop Nr." << paddedIndex << "] ICP fitness test passed (" << fitness
                      << " > " << loopFitnessScoreThreshold_Open3D_ << "). Adding this SC loop. \033[0m" << std::endl;
        } else {
            std::cout << "  [BAD  - SC loop Nr." << paddedIndex << "] ICP fitness test failed (" << fitness << " < "
                      << loopFitnessScoreThreshold_Open3D_ << "). Rejecting this SC loop." << std::endl;
        }
    }

}

std::tuple<Eigen::Matrix4d, double> PerformGICPwithOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_queryKeyframeCloud, const std::shared_ptr<open3d::geometry::PointCloud>& open3d_centralKeyframeCloud, const int version) {
    auto source_tree = std::make_shared<geometry::KDTreeFlann>(*open3d_queryKeyframeCloud);
    auto target_tree = std::make_shared<geometry::KDTreeFlann>(*open3d_centralKeyframeCloud);


    // Params
    double radius = 1;
    int max_nn = 30;

    open3d_queryKeyframeCloud->EstimateCovariances(geometry::KDTreeSearchParamHybrid(radius,max_nn));
    open3d_centralKeyframeCloud->EstimateCovariances(geometry::KDTreeSearchParamHybrid(radius, max_nn));
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d result_matrix;
    double fitness, inlier_rmse;

    if(version == 3)
    {
        std::tie(result_matrix, fitness, inlier_rmse) = PointCloudRegistration::make_icp_v5_open3D(*open3d_queryKeyframeCloud,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 100, 1, false, false);
    }
    else if(version == 4)
    {
        std::tie(result_matrix, fitness, inlier_rmse)  = PointCloudRegistration::make_icp_v6_only_yaw(*open3d_queryKeyframeCloud,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 100, 1, false, false);

        std::shared_ptr<geometry::PointCloud> aligned = std::make_shared<geometry::PointCloud>();
        *aligned = *open3d_queryKeyframeCloud;
        aligned->Transform(result_matrix);
        Eigen::Matrix4d result_matrix2;
        // Here only 10 iterations AND with a max_dist of 0.1 this will adjust the fitness calculation! -> leaving out wrongly registered point cluuds with a fitness < 0.3
        std::tie(result_matrix2, fitness, inlier_rmse) = PointCloudRegistration::make_icp_v5_open3D(*aligned,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 10, 0.1, false, false);

        result_matrix = result_matrix2 * result_matrix;
    }
    else
    {
    cout << " Give a valid version for ICP: 3 or 4"<< endl;
    }


    return  std::make_tuple(result_matrix, fitness);
}


// Function to pad a number on the left with a specified character
std::string pad_on_left(const size_t index, size_t total_length, char padding_char)
{
    std::string paddedIndex = std::to_string(index + 1);
    paddedIndex = std::string(total_length - paddedIndex.length(), padding_char) + paddedIndex;
    return paddedIndex;
}


// Function that adjusts the cloud to local coordinates
void adjustCentralCloudToLocalCoordinates(pcl::PointCloud<PointType>::Ptr centralKeyframeCloud, Session &target_sess, const int &loop_idx_target_session)
{
    // Translate the source point cloud to the position of the central point cloud in global coordinate system -> to be able to perform ICP with a well-constructed point cloud -> not as done in VirtualICP after SC loop detection
    float translation_x = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].x;
    float translation_y = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].y;
    float translation_z = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].z;

    float r_yaw = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].yaw;
    float r_pitch = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].pitch;
    float r_roll = target_sess.cloudKeyPoses6D->points[loop_idx_target_session].roll;

    // the central scan is moved (with 5 scans from the simulated scans) from the global to the local coord system -> where the central scan is at 0,0,0
    PcTransformations::transformPointCloudWithAngles(std::move(centralKeyframeCloud), -translation_x, -translation_y, -translation_z, -r_yaw, -r_pitch, -r_roll);

}

std::optional<gtsam::Pose3> Slam2ref::doICPVirtualRelative_v3_and_v4(Session &target_sess, Session &source_sess, const int &loop_idx_target_session, const int &loop_idx_source_session, const float &yaw_angle_source, const size_t &index, const int version) {
    mtx.lock();

    pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());

    source_sess.loopFindNearKeyframesGeneral(queryKeyframeCloud, loop_idx_source_session, 0,"local");
    target_sess.loopFindNearKeyframesGeneral(centralKeyframeCloud, loop_idx_target_session, NNKeyframes_for_submap_, "central");

    std::string paddedIndex = pad_on_left(index + 1, 3, '0');

    rotatePointCloud(queryKeyframeCloud, yaw_angle_source);

    adjustCentralCloudToLocalCoordinates(centralKeyframeCloud, target_sess, loop_idx_target_session);

    // Color the point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_centralKeyframeCloud = colorPointCloud(centralKeyframeCloud, green_color_utils);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_queryKeyframeCloud = colorPointCloud(queryKeyframeCloud, red_color_utils);

    // Transform PCL point cloud to Open3D
    auto open3dQueryKeyframeCloud = pclToOpen3d_v2(queryKeyframeCloud);
    auto open3dCentralKeyframeCloud = pclToOpen3d_v2(centralKeyframeCloud);
    std::shared_ptr<open3d::geometry::PointCloud> aligned = std::make_shared<open3d::geometry::PointCloud>();

    mtx.unlock();


    auto[resultMatrix, fitness] = PointCloudRegistration::PerformYawGICP(
            open3dQueryKeyframeCloud, open3dCentralKeyframeCloud, params->max_dist_thr_SC_ICP_, params->max_dist_thr_SC_ICP_2_, version);

    mtx.lock();
    bool passed = fitness > loopFitnessScoreThreshold_Open3D_;
    LogICPFitnessTestStatus_v2(print_SC_ICP_test_values_for_each_iteration_,loopFitnessScoreThreshold_Open3D_, passed, paddedIndex, fitness);

    if (passed) {
        *aligned = *open3dQueryKeyframeCloud;
        aligned->Transform(resultMatrix);
        colorOpen3DPointCloud(*aligned, blue_color_utils);

        SaveCloudsToFile(save_directory_, paddedIndex, aligned, colored_centralKeyframeCloud, colored_queryKeyframeCloud);
        mtx.unlock();
    } else {
        mtx.unlock();
        return std::nullopt;
    }

    return getPoseTransformation(resultMatrix);
} // doICPVirtualRelative_v3_and_v4


// While the next function is not used, it was refactored in a nice way;
// perhaps some of the sub-functions used can be used on the oder doICPVirtual functions
std::optional<gtsam::Pose3> Slam2ref::doICPVirtualGlobal_v3_for_KNN(Session &target_sess, Session &source_sess, const int &loop_idx_target_session, const int &loop_idx_source_session, const size_t &index, const int version) {
    mtx.lock();

    pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());

    source_sess.loopFindNearKeyframesGeneral(queryKeyframeCloud, loop_idx_source_session, 0,"local");
    target_sess.loopFindNearKeyframesGeneral(centralKeyframeCloud, loop_idx_target_session, NNKeyframes_for_submap_, "central");

    std::string paddedIndex = pad_on_left(index + 1, 3, '0');

    adjustCentralCloudToLocalCoordinates(centralKeyframeCloud, target_sess, loop_idx_target_session);

    // Color the point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_centralKeyframeCloud = colorPointCloud(centralKeyframeCloud, green_color_utils);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_queryKeyframeCloud = colorPointCloud(queryKeyframeCloud, red_color_utils);

    // Transform PCL point cloud to Open3D
    auto open3dQueryKeyframeCloud = pclToOpen3d_v2(queryKeyframeCloud);
    auto open3dCentralKeyframeCloud = pclToOpen3d_v2(centralKeyframeCloud);
    std::shared_ptr<open3d::geometry::PointCloud> aligned = std::make_shared<open3d::geometry::PointCloud>();

    mtx.unlock();

    auto[resultMatrix, fitness] = PerformGICPwithOpen3D(open3dQueryKeyframeCloud, open3dCentralKeyframeCloud, version);

    mtx.lock();
    bool passed = fitness > loopFitnessScoreThreshold_Open3D_;
    LogICPFitnessTestStatus_v2(print_SC_ICP_test_values_for_each_iteration_,loopFitnessScoreThreshold_Open3D_, passed, paddedIndex, fitness);

    if (passed) {
        *aligned = *open3dQueryKeyframeCloud;
        aligned->Transform(resultMatrix);
        colorOpen3DPointCloud(*aligned, blue_color_utils);

        SaveCloudsToFile(save_directory_, paddedIndex, aligned, colored_centralKeyframeCloud, colored_queryKeyframeCloud);
        mtx.unlock();
    } else {
        mtx.unlock();
        return std::nullopt;
    }

    return getPoseTransformation(resultMatrix);
} // doICPVirtualGlobal_v3_for_KNN

void createDirectories(const std::vector<std::string>& directories) {
    for(const auto& directory : directories) {
        std::filesystem::create_directories(directory);
    }
}


// Define a helper function to calculate the distance between poses
double calculateDistance(const PointTypePose &pose1, const PointTypePose &pose2) {
    return sqrt(pow(pose1.x-pose2.x, 2) + pow(pose1.y-pose2.y, 2) + pow(pose1.z-pose2.z, 2));
}


bool isPointWithinLimits(const Eigen::Vector3f& point,
                         const pcl::PointXYZ& minPt,
                         const pcl::PointXYZ& maxPt) {
    return (point.x() >= minPt.x && point.x() <= maxPt.x &&
            point.y() >= minPt.y && point.y() <= maxPt.y &&
            point.z() >= minPt.z && point.z() <= maxPt.z);
}

void colorAndSaveO3dPointCloud(std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud, const Eigen::Vector3i& color_utils, const std::string& file_path, std::unique_lock<std::mutex>& lock)
{
    colorOpen3DPointCloud(*open3d_cloud, color_utils);
    bool write_success = io::WritePointCloud(file_path, *open3d_cloud);
    if(!write_success) {
        lock.lock();
        std::cerr << "Failed to write the point cloud to: " << file_path << std::endl;
        lock.unlock();
    }
}


std::string padIndex(int index, int paddingLength = 6) {
    assert(paddingLength >= 0 && "paddingLength must be non-negative");
    std::string indexStr = std::to_string(index);
    if (indexStr.size() >= static_cast<std::string::size_type>(paddingLength)) {
        return indexStr;
    } else {
        std::string padding(paddingLength - indexStr.length(),'0');
        padding += indexStr;
        return padding;
    }
}

// This was an attempt to optimize a bit the parallelization -> select the number of poses per group according to the number of threads, so that there are less idle threads as possible -> this is however irrelevant if the poses are spaced more than 0.12 m (2 m / NrOfCores) since there will be idle threads anyway.
std::pair<std::vector<std::vector<PointTypePose>>, std::vector<double>> createGroupsAndCalcOffsets(const pcl::PointCloud<PointTypePose>::Ptr &cloudKeyPoses6D, const int numberOfCores, const double offsetBetweenPoseGroups) {
    std::vector<std::vector<PointTypePose>> poseGroups;
    std::vector<double> offsetBetweenPoseGroupsPerGroup;
    auto it = cloudKeyPoses6D->points.begin();
    while (it != cloudKeyPoses6D->points.end()) {
        std::vector<PointTypePose> group;
        PointTypePose* firstPoseInGroup = &(*it);
        group.push_back(*it);
        for (++it; it != cloudKeyPoses6D->points.end(); ++it) {
            int groupSize = group.size();
            if ((groupSize % numberOfCores != 0 && groupSize > numberOfCores) || groupSize >= numberOfCores)
                break;
            if(calculateDistance(*firstPoseInGroup, *it) <= offsetBetweenPoseGroups)
                group.push_back(*it);
        }
        while (int groupSize = group.size()) { // initializes the groupSize variable and runs as long as the group has at least one member (since .size() returns 0 for an empty group which breaks the loop).
            if (groupSize > numberOfCores && groupSize % numberOfCores != 0) {
                group.pop_back();
                --it;
            } else if (it != cloudKeyPoses6D->points.end() && groupSize < numberOfCores && groupSize % numberOfCores != 0) {
                group.push_back(*it++);
            } else {
                break;
            }
        }
        if (!group.empty() && group.size() > 1)
            offsetBetweenPoseGroupsPerGroup.push_back(calculateDistance(group.front(), group.back()));
        poseGroups.push_back(group);
    }
    return std::make_pair(poseGroups, offsetBetweenPoseGroupsPerGroup);
}


void printGroupDetails(const std::vector<std::vector<PointTypePose>>& poseGroups, const std::vector<double>& offsetBetweenPoseGroupsPerGroup) {
    if(poseGroups.size() != offsetBetweenPoseGroupsPerGroup.size()) {
        std::cout << "Error: The number of pose groups and offsets do not match." << std::endl;
        return;
    }
    std::cout << "The number of poses in the groups and offsets are:" << std::endl;
    for (size_t i = 0; i < poseGroups.size(); ++i) {
        std::cout << "\nGroup Nr. " << i + 1
                  << ", size: " << poseGroups[i].size()
                  << ", offset: " << offsetBetweenPoseGroupsPerGroup[i]
                  << std::endl;
    }
    std::cout << "\n" << std::endl;
}

// This is the same as v2 but refactored
void Slam2ref::performFinalIcpWithOriginalMap_v3()
{
    std::cout << "Doing ICP of all the scans in the query session with the corresponding submap of the reference point cloud" << std::endl;
    auto &source_sess = sessions_.at(query_sess_idx);

    // Print the number of scans
    std::size_t num_scans_in_query_sess = source_sess.nodes_.size();
    std::cout << "num of scans to be registered: " << num_scans_in_query_sess << std::endl;

    // Prepare target point cloud
    pcl::PointCloud<PointType>::Ptr target_cloud(new pcl::PointCloud<PointType>());
    loadPointCloud(params->path_to_target_pc_for_final_ICP_, target_cloud);
    downsamplePointCloud(target_cloud, 0.01f);  // 1 cm downsampling for the target, reference pc.
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*target_cloud, minPt, maxPt);

    std::vector<std::pair<int, gtsam::Pose3>> pairs_of_final_poses;

    // Prepare file directories
    std::string dir = save_directory_ + "Part7_final_ICP/";
    std::string dir_final_very_good = dir + "01_VERY_GOOD_source_scans_after_ICP/";
    std::string dir_final_good = dir + "02_GOOD_source_scans_after_ICP/";
    std::string dir_final_bad = dir + "03_BAD_source_scans_after_ICP/";
    std::string dir_final_outside = dir + "04_Outside_source_scans_after_ICP/";
    createDirectories({dir, dir_final_very_good, dir_final_good, dir_final_bad, dir_final_outside});

    // Prepare files
    std::string filename_central = dir + "!00_CENTRAL_SESSION_after_FinalICP.txt";
    std::fstream stream_central_inv(filename_central.c_str(), std::fstream::out);

    pcl::PointCloud<PointType>::Ptr finalCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud_sphere(new pcl::PointCloud<PointType>());
    Eigen::Vector3f previousCenter;
    std::shared_ptr<open3d::geometry::PointCloud> open3d_centralKeyframeCloud = std::make_shared<open3d::geometry::PointCloud>();

    // Create a vector for storing <index, category> pairs.
    std::vector<std::pair<int, int>> poseCategoryVector;
    std::vector<std::vector<PointTypePose>> poseGroups;
    double offsetBetweenPoseGroups = 2;// 2 m

    // BEFORE
    for (auto it = source_sess.cloudKeyPoses6D->points.begin(); it != source_sess.cloudKeyPoses6D->points.end();) {
        std::vector<PointTypePose> group;
        PointTypePose* firstPoseInGroup = &(*it);
        group.push_back(*it);

        // Check if there is a next item in the array and if it should be added to the current group
        for (++it; it != source_sess.cloudKeyPoses6D->points.end() && calculateDistance(*firstPoseInGroup, *it) <= offsetBetweenPoseGroups; ++it) {
            // If it should, add it to the current group
            group.push_back(*it);
        }

        // Add the current group to the groups of poses
        poseGroups.push_back(group);
    }
//    // After
//    std::vector<double> offsetBetweenPoseGroupsPerGroup;
//    int min_number_of_poses_per_group = params->numberOfCores_;
//    auto result_groups = createGroupsAndCalcOffsets(source_sess.cloudKeyPoses6D, min_number_of_poses_per_group, offsetBetweenPoseGroups);
//    poseGroups = result_groups.first;
//    offsetBetweenPoseGroupsPerGroup = result_groups.second;
//    printGroupDetails(poseGroups, offsetBetweenPoseGroupsPerGroup);

    int poseGroup_index = 0;
    int pose_index = 0;
    int last_pose_index_in_group = 0;
    std::string fileNameAndExtension = "_source_cloud_after_FINAL_ICP.pcd";
    // Perform operations on the pose groups

    auto startIndexGroup = static_cast<std::vector<decltype(poseGroups)>::size_type>(params->start_group_index_Final_ICP_);


    int accumulatedIndex = 0;

    if (startIndexGroup > 0 && startIndexGroup < poseGroups.size()) {
        accumulatedIndex = std::accumulate(poseGroups.begin(), poseGroups.begin() + startIndexGroup, 0, [](int acc, const std::vector<PointTypePose>& vec) {
                    return acc + vec.size();
                });
    }
    bool first_time = true;
    for (size_t j = startIndexGroup; j < poseGroups.size(); ++j) {
        const auto& poseGroup = poseGroups[j];
        poseGroup_index = j;
        std::cout << "\n \n========= Group Index: "<<poseGroup_index <<" =========== \n"<<std::endl;

         // compute the center as the average of the first and last pose
        PointTypePose firstPose = poseGroup.front();
        PointTypePose lastPose = poseGroup.back();

        Eigen::Vector3f firstPoseCenter(firstPose.x, firstPose.y, firstPose.z);
        Eigen::Vector3f lastPoseCenter(lastPose.x, lastPose.y, lastPose.z);

        // Average the coordinates
        Eigen::Vector3f center = (firstPoseCenter + lastPoseCenter) / 2.0;
        if(!isPointWithinLimits(center, minPt, maxPt)) {
            std::cout << "The center of this group does not lie within reference cloud bounds, therefore the group should not be processed" << std::endl; // TODO complete
        }

        double sphere_radius = params->PC_MAX_RADIUS_ + offsetBetweenPoseGroups;
        centralKeyframeCloud_sphere = cropPointCloudWithinSphere(target_cloud, center, sphere_radius, numberOfCores_);

        pclToOpen3d(centralKeyframeCloud_sphere, open3d_centralKeyframeCloud);
        std::cout << "\n Size of open3d_centralKeyframeCloud: "<< open3d_centralKeyframeCloud->points_.size() << std::endl;

        std::string paddedIndex_group = padIndex(poseGroup_index, 6);

        if(params->save_all_source_and_target_scans_in_final_ICP_) {
            if (centralKeyframeCloud_sphere->empty()) {
                std::cout << "centralKeyframeCloud_sphere is empty!\n";
            } else {
                int result = pcl::io::savePCDFileBinary(dir + paddedIndex_group + "_target_scan.pcd",*centralKeyframeCloud_sphere);
                if (result != 0) {  // If save operation was not successful, 'result' contains the error code
                    std::cerr << "  Error: Failed to save the PCD file. Error code: " << result << std::endl;
                } else {
                    std::cout << "  Success: Central PCD file saved successfully." << std::endl;
                }
            }
        }

        if(first_time){
            pose_index = accumulatedIndex;
            last_pose_index_in_group = static_cast<int>(poseGroup.size()) + pose_index;
            first_time = false;
        }
        else{
            pose_index = last_pose_index_in_group;
            last_pose_index_in_group += static_cast<int>(poseGroup.size());
        }
        // Couts for debbugging
//        std::cout << "      pose_index IN GROUP; "<<pose_index << std::endl;
//        std::cout << "      last_pose_index_in_group: "<< last_pose_index_in_group << std::endl;

        // Iterate over the poses in group, do parallel calculations
#pragma omp parallel for  num_threads(numberOfCores_) default(none) shared(num_scans_in_query_sess, source_sess, target_cloud, cout, pairs_of_final_poses, dir, dir_final_very_good, dir_final_good,dir_final_bad, finalCloud, open3d_centralKeyframeCloud, poseGroup, cerr, poseGroup_index, dir_final_outside, pose_index, last_pose_index_in_group, centralKeyframeCloud_sphere, fileNameAndExtension, poseCategoryVector)

        for (auto k = pose_index; k < last_pose_index_in_group; ++k) {
            int pose_category;// 4 Outside of ref map, 3 bad FS, 2 Good FS, 1 Very good.

            // Do calculations for each pose in the group (ICP)

            std::unique_lock<std::mutex> lock(mtx);
            std::ostringstream os;   // Use a separate ostringstream
            os << "         pose_index in parallel For: " << k << std::endl;
            std::cout << os.str();  // Output all at once

            PointTypePose* pose_init = &source_sess.cloudKeyPoses6D->points[k];


            pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr aligned_pcl(new pcl::PointCloud<PointType>());
            lock.unlock();


            // Source point cloud in the final calculated pose
            *queryKeyframeCloud += *transformPointCloud(source_sess.cloudKeyFrames[k], pose_init);


            lock.lock();

            std::string paddedIndex = padIndex(k, 6);

            if(params->save_all_source_and_target_scans_in_final_ICP_) {
                if (queryKeyframeCloud->empty()) {
                    std::cout << "queryKeyframeCloud is empty!\n";
                } else {
                    int result = pcl::io::savePCDFileBinary(dir + paddedIndex + "_source_initial_scan_after_1st_T.pcd", *queryKeyframeCloud);

                    if (result != 0) {  // If save operation was not successful, 'result' contains the error code
                        std::cerr << "  Error: Failed to save the PCD file. Error code: " << result << std::endl;
                    }
//                    else {
//                        std::cout << "  Success: Query PCD file saved successfully." << std::endl;
//                    }
                }
            }

            // Transform PCL point cloud to Open3D
            std::shared_ptr<open3d::geometry::PointCloud> open3d_queryKeyframeCloud = std::make_shared<open3d::geometry::PointCloud>();
            std::shared_ptr<open3d::geometry::PointCloud> aligned = std::make_shared<open3d::geometry::PointCloud>();

            pclToOpen3d(queryKeyframeCloud, open3d_queryKeyframeCloud);
            if (open3d_centralKeyframeCloud->IsEmpty()) {
                // If the fitness score is lower,
                // it is because the source point cloud is outsite of the reference map or is very distorted ->
                // therefore, the original pose will be considered instead,
                // and the scan will be saved in another folder with red color
                std::cout << "open3d_centralKeyframeCloud is EMPTY -> registration will be skipped only original poses will be saved"<< std::endl;
                if (params->save_registered_source_scans_in_final_ICP_)
                {
                    colorOpen3DPointCloud(*open3d_queryKeyframeCloud, black_color_utils);
                    auto file_path = dir_final_outside + paddedIndex  + "_000" +fileNameAndExtension;
                    bool write_success = io::WritePointCloud(file_path, *open3d_queryKeyframeCloud);
                    if (!write_success) {
                        std::cerr << "Failed to write Outside point cloud to: " << file_path << std::endl;
                    }
                }

                gtsam::Pose3 pose_init_gtsam = Pose3(Rot3::RzRyRx(pose_init->roll, pose_init->pitch, pose_init->yaw), Point3(pose_init->x, pose_init->y, pose_init->z));
                pairs_of_final_poses.push_back(std::make_pair(k,pose_init_gtsam));
                pose_category = 4; // 4 Outside of ref map, 3 bad FS, 2 Good FS, 1 Very good.
                poseCategoryVector.push_back({k, pose_category});
                lock.unlock();// Unlock Mutex before a continue statement
                continue;
            }


            lock.unlock();

            // Performing P2P ICP in three phases
            // Create KD Trees for source and target

            Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
            auto[result_matrix1, fitness1, inlier_rmse1] = PointCloudRegistration::make_icp_v8_p2p(
                    *open3d_queryKeyframeCloud, *open3d_centralKeyframeCloud, *open3d_centralKeyframeCloud, initial_guess,
                    100, max_dist_thr_Final_ICP_, false, false, true);

            std::shared_ptr<geometry::PointCloud> aligned2 = std::make_shared<geometry::PointCloud>();
            *aligned2 = *open3d_queryKeyframeCloud;
            aligned2->Transform(result_matrix1);

            // Here only 100 iterations AND with a max_dist of 0.01 (1 cm) this will adjust the fitness calculation!
            auto[result_matrix2, fitness2, inlier_rmse2] = PointCloudRegistration::make_icp_v8_p2p(*aligned2,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 100, params->max_dist_thr_Final_ICP_2, false, false, false); // the second we do not downsmample source, to have a better FS calculation
            auto result_matrix_2_temp = result_matrix2 * result_matrix1;


            std::shared_ptr<geometry::PointCloud> aligned3 = std::make_shared<geometry::PointCloud>();
            *aligned3 = *open3d_queryKeyframeCloud;
            aligned3->Transform(result_matrix_2_temp);

            // Here only 10 iterations AND with a max_dist of 0.01 (1 cm) this will adjust the fitness calculation!
            auto[result_matrix3, fitness3, inlier_rmse3] = PointCloudRegistration::make_icp_v8_p2p(*aligned3,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 10, params->max_dist_thr_Final_ICP_3, false, false, false); // again here we do not downsmample source, to have a better FS calculation

            auto result_matrix = result_matrix3* result_matrix_2_temp;

            *aligned = *open3d_queryKeyframeCloud;
            aligned->Transform(result_matrix);


            double max_correspondence_distance = 0.01;
            auto [fitness_1cm, fitness_1cm_v2, rmse, rmse_InRange] = PointCloudRegistration::EvaluateRegistrationAlignedSourceAndRMSE(*aligned,*open3d_centralKeyframeCloud,max_correspondence_distance,params->range_distance_for_second_fitness_);

            max_correspondence_distance = 0.03;
            auto [fitness_3cm, fitness_3cm_v2, rmse_3cm, rmse_3cmInRange] = PointCloudRegistration::EvaluateRegistrationAlignedSourceAndRMSE(*aligned,*open3d_centralKeyframeCloud,max_correspondence_distance,params->range_distance_for_second_fitness_);

            // Print transformation obtained by ICP
            std::ostringstream os2;   // Use a separate ostringstream
            os2 << "Scan: " << k << std::endl;
            os2 << std::fixed << std::setprecision(3);
            os2 << "  ICP RMSE:          " << rmse << std::endl;
            os2 << "  ICP RMSE in_range: " << rmse_InRange << std::endl;
            os2 << "  ICP fitness score 1cm:          " << fitness_1cm << std::endl;
            os2 << "  ICP fitness score 1cm in_range: " << fitness_1cm_v2 << std::endl;
//            os2 << "  ICP RMSE 1cm in_range : " << rmse_1cmInRange << std::endl;

            os2 << "  ICP fitness score 3cm:          " << fitness_3cm << std::endl;
            os2 << "  ICP fitness score 3cm in_range: " << fitness_3cm_v2 << std::endl;
//            os2 << "  ICP RMSE 3cm RMSE:      " << rmse_3cm << std::endl;
//            os2 << "  ICP RMSE 3cm in_range:  " << rmse_3cmInRange << std::endl;

            {
                lock.lock();
                std::cout << os2.str();
                lock.unlock();
            }
            double paddedFitness = fitness_1cm_v2 * 100;  // move the decimal point
            std::stringstream stream;
            stream << std::fixed << std::setprecision(0) << paddedFitness;
            std::string paddedFitness_str = stream.str();
            paddedFitness_str = std::string(3 - paddedFitness_str.length(), '0') + paddedFitness_str;

            if(fitness_1cm_v2>params->loopFitnessScoreThreshold_Final_ICP_ && fitness_3cm_v2>params->loopFitnessScoreThreshold_Final_ICP_3cm_ && rmse_InRange < params->loopRmseThreshold_Final_ICP_ ){
                //VERY GOOD point cloud registration (first thr)

                if (params->save_registered_source_scans_in_final_ICP_)
                {
                    auto file_path = dir_final_very_good + paddedIndex + "_" + paddedFitness_str + fileNameAndExtension;
                    colorAndSaveO3dPointCloud(aligned, blue_color_utils, file_path, lock);

                }
                open3dToPcl(aligned, aligned_pcl);
                // Accumulating the point cloud
                *finalCloud += *aligned_pcl;

                // Part 6: Get pose transformation and return the relative pose
                float x, y, z, roll, pitch, yaw;
                Eigen::Affine3f correctionLidarFrame = static_cast<Affine3f>(result_matrix.cast<float>());
                pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 gicp_pose = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

                // The following two lines are only to write poses
                gtsam::Pose3 pose_init_gtsam = Pose3(Rot3::RzRyRx(pose_init->roll, pose_init->pitch, pose_init->yaw), Point3(pose_init->x, pose_init->y, pose_init->z));

                Pose3 pose_final2 =  gicp_pose.compose(pose_init_gtsam); // This is correct, basically; first to the pose_init_gtsam transformation the icp_pose transform will be added, the opposite will be wrong (see pose_final above)
                pairs_of_final_poses.push_back(std::make_pair(k,pose_final2));
                pose_category = 1; // 4 Outside of ref map, 3 bad FS, 2 Good FS, 1 Very good.
            }
            else if(fitness_1cm_v2> params->loopFitnessScoreThreshold_Final_ICP_2_ && fitness_3cm_v2>params->loopFitnessScoreThreshold_Final_ICP_3cm_2_ && rmse_InRange < params->loopRmseThreshold_Final_ICP_2_){
                // GOOD registration (second thr)
                if (params->save_registered_source_scans_in_final_ICP_)
                {
                    auto file_path = dir_final_good + paddedIndex + "_" + paddedFitness_str + fileNameAndExtension;
                    colorAndSaveO3dPointCloud(aligned, blue_color_utils, file_path, lock);

                }
                open3dToPcl(aligned, aligned_pcl);
                // Accumulating the point cloud
                *finalCloud += *aligned_pcl;

                // Part 6: Get pose transformation and return the relative pose
                float x, y, z, roll, pitch, yaw;
                Eigen::Affine3f correctionLidarFrame = static_cast<Affine3f>(result_matrix.cast<float>());
                pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 gicp_pose = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

                // The following two lines are only to write poses
                gtsam::Pose3 pose_init_gtsam = Pose3(Rot3::RzRyRx(pose_init->roll, pose_init->pitch, pose_init->yaw), Point3(pose_init->x, pose_init->y, pose_init->z));

                Pose3 pose_final2 =  gicp_pose.compose(pose_init_gtsam);
                pairs_of_final_poses.push_back(std::make_pair(k,pose_final2));
                pose_category = 2; // 4 Outside of ref map, 3 bad FS, 2 Good FS, 1 Very good.
            }
            else{
                // If the fitness score is lower, it is because the source point cloud is outside of the reference map or is very distorted -> therefore, the original pose will be considered instead, and the scan (after the final ICP) will be saved in another folder with red color

                if (params->save_registered_source_scans_in_final_ICP_)
                {
                    std::string path = dir_final_bad + paddedIndex  + "_" + paddedFitness_str + fileNameAndExtension;
                    colorAndSaveO3dPointCloud(aligned, red_color_utils, path, lock);
                }

                gtsam::Pose3 pose_init_gtsam = Pose3(Rot3::RzRyRx(pose_init->roll, pose_init->pitch, pose_init->yaw), Point3(pose_init->x, pose_init->y, pose_init->z));
                pairs_of_final_poses.push_back(std::make_pair(k,pose_init_gtsam));
                pose_category = 3; // 4 Outside of ref map, 3 bad FS, 2 Good FS, 1 Very good.

            }

            lock.lock();
            poseCategoryVector.push_back({k, pose_category});
            lock.unlock();
        }
    }

// Custom comparator function to compare pairs based on the first element (index) -> to write the final poses in the correct order
    auto comparator = [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    };

    // Sort the vector by indices using the custom comparator
    std::sort(pairs_of_final_poses.begin(), pairs_of_final_poses.end(), comparator);
    // Write the poses in a txt file
    for (auto & pairs_of_final_pose : pairs_of_final_poses) {
        writePose3ToStream(stream_central_inv, pairs_of_final_pose.second);
    }


    auto leaf_size = 0.05f;
    downsamplePointCloud(finalCloud, leaf_size);

    // Write final down sampled final Cloud
    // Check if finalCloud contains points
    if (!finalCloud->empty()) {
        pcl::io::savePCDFileBinary(dir + "!FINAL_source_cloud_TOTAL.pcd", *finalCloud);
        std::cout << "Point cloud saved." << std::endl;
    }
    else {
        // If the cloud is empty, we still want to write a file. This will be an empty PCD file.
        std::cout << "Final point cloud is empty (i.e. not scans are pass the good thrs), it will not be saved." << std::endl;
    }
    // Save category info to a text file
    std::sort(poseCategoryVector.begin(), poseCategoryVector.end());
    std::ofstream outputFile(dir + "list_of_scan_categories.txt");
    outputFile << "PoseIndex Category\n";
    if(outputFile.is_open())
    {
        for(const auto &item : poseCategoryVector) {
//            outputFile << "Pose: " <<item.first<<", Category: "<<item.second<< "\n";
            outputFile << item.first << " " << item.second << "\n";
        }

        outputFile.close();
        std::cout << "Saved the results to list_of_scan_categories.txt\n";
    }
    else
        std::cout << "Unable to open output.txt to write.";

    std::string filename_cc_color = dir + "!00_CENTRAL_SESSION_after_FinalICP_CC.txt";
    std::fstream stream_cc_poses_with_color(filename_cc_color.c_str(), std::fstream::out);
    writeAllPosesWithColorsForCC(stream_cc_poses_with_color, pairs_of_final_poses, poseCategoryVector);
} // performFinalICP_withOriginalMap



void writeIndicesToFile(const std::vector<int>& indices, const std::string& filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
        for (int index : indices) {
            file << index << std::endl;
        }

        std::cout << "Indices written to " << filename << std::endl;
        file.close();
    } else {
        std::cerr << "Unable to open the file: " << filename << std::endl;
    }
}


/* in contrast to the previous version (v1), this function
 * Creates a point cloud from the scans of the central session that are close to the position of the query session that I want to improve, and use it as a target point cloud instead of a point cloud created from BIM
 * -> doing this I will be able to leave out points outside the field of view
 * -> to make a better ICP registration.
 * */

/* in contrast to the second version (v2), this function
 * Uses the GICP from open3D in a for loop, this way seems to be the most promising in comparison with PCL and NANO-GICP.
 * */
void Slam2ref::performFinalICP_v3()
{
    std::cout << "Doing ICP of all the scans in the query session with the corresponding submap of the central session" << std::endl;
    // add selected rs loops
    auto &target_sess = sessions_.at(central_sess_idx);
    auto &source_sess = sessions_.at(query_sess_idx);

    std::size_t num_scans_in_query_sess = source_sess.nodes_.size();
//    std::string dir = "/home/mlegion/ws/07_sc_a_loam/src/data/05.FinalICP_tests/";
    std::string dir = save_directory_ + "Part7_final_ICP/";
    std::filesystem::create_directories(dir);

    std::string dir_final = dir + "FINAL_source_scans_after_ICP/";
    std::filesystem::create_directories(dir_final);

    cout << "num of scans to be registered: " << num_scans_in_query_sess<< endl;


    std::string filename_central_inv = dir + "!00_CENTRAL_SESSION_after_FinalICP_INVERTED.txt";
    std::fstream stream_central_inv(filename_central_inv.c_str(), std::fstream::out);


    int number_of_k_nearest_poses_to_find = NNKeyframes_for_submap_;

    std::vector<std::pair<int, gtsam::Pose3>> pairs_of_final_poses2;

    // Convert PointCloud<PointTypePose> to PointCloud<pcl::PointXYZ> needed for KNN search
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloudXYZ = convertPointCloud(target_sess.cloudKeyPoses6D);

    pcl::PointCloud<PointType>::Ptr finalCloud(new pcl::PointCloud<PointType>());



#pragma omp parallel for  num_threads(numberOfCores_) default(none) shared(num_scans_in_query_sess, source_sess, target_sess, target_cloudXYZ, cout, number_of_k_nearest_poses_to_find, dir, finalCloud, pairs_of_final_poses2, dir_final) // stream_central, stream_pose_init_gtsam, stream_icp,pairs_of_final_poses
    for (auto k = 0; k < static_cast<int>(num_scans_in_query_sess); k++) {
        mtx.lock();
        // Source pose
        PointTypePose* pose_init = &source_sess.cloudKeyPoses6D->points[k];


        pcl::PointXYZ pose_init_XYZ = transformPoint(*pose_init);

        // Call the function to find k-nearest points
        std::vector<int> nearestIndices = findKNearestPoints(target_cloudXYZ, pose_init_XYZ, number_of_k_nearest_poses_to_find);


        pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr aligned_pcl(new pcl::PointCloud<PointType>());

        target_sess.createPointCloudWithIndices(centralKeyframeCloud, nearestIndices, "central");

        std::string paddedIndex = std::to_string(k + 1);
        paddedIndex = std::string(3 - paddedIndex.length(), '0') + paddedIndex;

        // source point cloud -> the source current scan transformed to global coord. sys. with the pose_init
        *queryKeyframeCloud += *transformPointCloud(source_sess.cloudKeyFrames[k], pose_init);

        if(params->save_all_source_and_target_scans_in_final_ICP_)
        {
            pcl::io::savePCDFile(dir + paddedIndex + "_source_initial_scan_after_1st_T.pcd", *queryKeyframeCloud);
            pcl::io::savePCDFile(dir + paddedIndex + "_target_scan.pcd", *centralKeyframeCloud);
        }

        // Transform PCL point cloud to Open3D
        std::shared_ptr<open3d::geometry::PointCloud> open3d_queryKeyframeCloud = std::make_shared<open3d::geometry::PointCloud>();
        std::shared_ptr<open3d::geometry::PointCloud> open3d_centralKeyframeCloud = std::make_shared<open3d::geometry::PointCloud>();
        std::shared_ptr<open3d::geometry::PointCloud> aligned = std::make_shared<open3d::geometry::PointCloud>();

        pclToOpen3d(queryKeyframeCloud, open3d_queryKeyframeCloud);
        pclToOpen3d(centralKeyframeCloud, open3d_centralKeyframeCloud);

        mtx.unlock();


        // Part 3: GICP with Open 3D

        // Create KD Trees for source and target
        auto source_tree = std::make_shared<geometry::KDTreeFlann>(*open3d_queryKeyframeCloud);
        auto target_tree = std::make_shared<geometry::KDTreeFlann>(*open3d_centralKeyframeCloud);

        // MV the following parameter (radius) is very important to achieve a good registration performance, in my case 0.02 did not work, but 1 did a very good job!
        double radius = 1;
        int max_nn = 30;
        open3d_queryKeyframeCloud->EstimateCovariances(geometry::KDTreeSearchParamHybrid(radius,max_nn));
        open3d_centralKeyframeCloud->EstimateCovariances(geometry::KDTreeSearchParamHybrid(radius, max_nn));

        Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
        auto[result_matrix, fitness, inlier_rmse] = PointCloudRegistration::make_icp_v5_open3D(
                *open3d_queryKeyframeCloud, *open3d_centralKeyframeCloud, *open3d_centralKeyframeCloud, initial_guess,
                100, max_dist_thr_Final_ICP_, false, false);

        mtx.lock();

        // Print transformation obtained by ICP
        std::cout << "GICP fitness score: " << fitness  << std::endl;

        *aligned = *open3d_queryKeyframeCloud;
        aligned->Transform(result_matrix);


        double paddedFitness = fitness * 100;  // move the decimal point

        std::stringstream stream;
        stream << std::fixed << std::setprecision(0) << paddedFitness;

        std::string paddedFitness_str = stream.str();

        if(params->save_registered_source_scans_in_final_ICP_)
        {
        colorOpen3DPointCloud(*aligned, blue_color_utils);
        io::WritePointCloud(dir_final + paddedFitness_str + "_" + paddedIndex +"_FINAL_source_cloud_after_FINAL_ICP.pcd", *aligned);
        }


        open3dToPcl(aligned, aligned_pcl);
        // Accumulating the point cloud
        *finalCloud += *aligned_pcl;

        // Part 6: Get pose transformation and return the relative pose
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame = static_cast<Affine3f>(result_matrix.cast<float>());
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 gicp_pose = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));

        // The following two lines are only to write poses
        gtsam::Pose3 pose_init_gtsam = Pose3(Rot3::RzRyRx(pose_init->roll, pose_init->pitch, pose_init->yaw), Point3(pose_init->x, pose_init->y, pose_init->z));

        Pose3 pose_final2 =  gicp_pose.compose(pose_init_gtsam); // This is correct, basically; first to the pose_init_gtsam transformation, the icp_pose transform will be added, the opposite will be wrong (see pose_final above)
        pairs_of_final_poses2.push_back(std::make_pair(k,pose_final2));

        mtx.unlock();
    }

    // Custom comparator function to compare pairs based on the first element (index) -> to write the final poses in the correct order
    auto comparator = [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    };


    // Sort the vector by indices using the custom comparator
    std::sort(pairs_of_final_poses2.begin(), pairs_of_final_poses2.end(), comparator);
    // Write the poses in a txt file
    for (auto & pairs_of_final_pose : pairs_of_final_poses2) {
        writePose3ToStream(stream_central_inv, pairs_of_final_pose.second);
    }

    // Create a VoxelGrid object
    pcl::VoxelGrid<PointType> voxel_grid_filter;

    // Set the input cloud to the filter
    voxel_grid_filter.setInputCloud(finalCloud);  // .makeShared() creates a shared pointer to the input cloud

    // Set the voxel grid leaf size (adjust this based on your desired downsampling level)
    auto leaf_size = 0.05f; //leaf size: 5 cm

    voxel_grid_filter.setLeafSize(leaf_size,leaf_size,leaf_size);
    // Call the filtering function to obtain the downsampled output
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());
    voxel_grid_filter.filter(*downsampled_cloud);

    // Write final down sampled Cloud
    pcl::io::savePCDFileBinary(dir + "!FINAL_source_cloud_TOTAL.pcd", *downsampled_cloud);


} // performFinalICP_v3

