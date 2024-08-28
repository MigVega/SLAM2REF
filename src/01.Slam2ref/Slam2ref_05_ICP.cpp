// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//


#include "01.Slam2ref/Slam2ref.h"

/**
 * @brief Performs Iterative Closest Point (ICP) based relative pose estimation for ScanContext loop closure.
 * 
 * This function conducts ICP between near keyframes in the local coordinate systems of the central and source sessions.
 * It aligns 1 scan from the source point cloud to "historyKeyframeSearchNum" scans from the central point cloud, computes the relative pose transformation,
 * and returns the relative pose as a gtsam::Pose3 object.
 *
 * @note The function saves point clouds before ICP for debugging purposes.
 *
 * @param central_sess The central session for the loop closure.
 * @param source_sess The source session for the loop closure.
 * @param loop_idx_central_session The loop index in the central session.
 * @param loop_idx_source_session The loop index in the source session.
 * @return An optional gtsam::Pose3 representing the relative pose transformation if successful, std::null opt otherwise.
 */
std::optional<gtsam::Pose3> Slam2ref::doICPVirtualRelative( // for SC loop
        Session &central_sess, Session &source_sess,
        const int &loop_idx_central_session, const int &loop_idx_source_session)
{
    // Part 1: Parse point clouds and initialize keyframe point cloud containers
    mtx.lock();
    pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());

    // Part 2: Retrieve keyframes in local coordinates from both central and source sessions
    source_sess.loopFindNearKeyframesLocalCoord(queryKeyframeCloud, loop_idx_source_session, 0);
    central_sess.loopFindNearKeyframesLocalCoord(centralKeyframeCloud, loop_idx_central_session, historyKeyframeSearchNum);
    mtx.unlock(); // unlock after loopFindNearKeyframesWithRespectTo because many new in the loopFindNearKeyframesWithRespectTo

    // Part 2.1 (MV): Save point clouds as PLY files for debugging
    // MV: Save point clouds as PLY files
    pcl::io::savePCDFileBinary("/home/mlegion/ws/09_bim_slam_mod_ltMapper/src/Slam2ref/data/00_central_cloud_ORIGINAL_virtual_ICP.pcd", *centralKeyframeCloud);
    pcl::io::savePCDFileBinary("/home/mlegion/ws/09_bim_slam_mod_ltMapper/src/Slam2ref/data/00_source_cloud_ORIGINAL_virtual_ICP.pcd", *queryKeyframeCloud);
//    logInfo_cout("\033[1;36m Point Clouds saved before ICP.\033[0m");

    // Part 3: ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(max_dist_thr_original_SC_ICP_); // giseop, use a value can cover 2*historyKeyframeSearchNum range in meter // MV: changed from 150 to 10 -> for indoor
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
            std::cout << "  [SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > "
                      << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
            mtx.unlock();
        }
        return std::nullopt;
    }
    else
    {
        if(print_SC_ICP_test_values_for_each_iteration_) {
            mtx.lock();
            std::cout << "  [SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < "
                      << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
            mtx.unlock();
        }
    }

    // Part 6: Get pose transformation and return the relative pose
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));


    if(use_original_WRONG_ICP_SC_result_)
        return poseFrom.between(poseTo); // MV: this is wrong
    else
        return poseTo.between(poseFrom); // MV: this is Correct
} // doICPVirtualRelative



std::optional<gtsam::Pose3> Slam2ref::doICPGlobalRelative( // For RS loop
        Session &central_sess, Session &source_sess,
        const int &loop_idx_central_session, const int &loop_idx_source_session)
{
    // parse pointclouds
    mtx.lock();
    pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());


    source_sess.loopFindNearKeyframesCentralCoord(queryKeyframeCloud, loop_idx_source_session, 0);
    central_sess.loopFindNearKeyframesCentralCoord(centralKeyframeCloud, loop_idx_central_session, historyKeyframeSearchNum);
    mtx.unlock();

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    if( queryKeyframeCloud->empty())
        std::cout << "  queryKeyframeCloud is empty, ICP alignment can not happend" << std::endl;
    icp.setInputSource(queryKeyframeCloud);
    icp.setInputTarget(centralKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);


    if (!icp.hasConverged() || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        if(print_SC_ICP_test_values_for_each_iteration_) {
            mtx.lock();
            std::cout << "  [BAD  - RS loop] ICP fitness test failed (" << icp.getFitnessScore() << " > "
                      << loopFitnessScoreThreshold << "). Reject this RS loop." << std::endl;
            mtx.unlock();
        }
        return std::nullopt;
    }
    else
    {
        if(print_SC_ICP_test_values_for_each_iteration_) {
            mtx.lock();
            std::cout << "\033[32m  [GOOD - RS loop] ICP fitness test passed (" << icp.getFitnessScore() << " < "
                      << loopFitnessScoreThreshold << "). Add this RS loop. \033[0m" << std::endl;
            mtx.unlock();
        }
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    Eigen::Affine3f tWrong = pclPointToAffine3f(source_sess.cloudKeyPoses6D->points[loop_idx_source_session]);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(central_sess.cloudKeyPoses6D->points[loop_idx_central_session]);

    if(use_original_WRONG_ICP_RS_result_)
        return poseFrom.between(poseTo); // MV: this is wrong
    else
        return poseTo.between(poseFrom); // MV: this is Correct
} // doICPGlobalRelative


