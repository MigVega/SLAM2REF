//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#include "01.Slam2ref/Slam2ref.h"
#include "0.Utils/PointCloudRegistration.h"
#include "0.Utils/PcTransformations.h"


void comparePoses(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2) {
    // Print the poses
    std::cout << "Pose1 (pose_final2): " << pose1 << std::endl;
    std::cout << "Pose2 (poseFrom): " << pose2 << std::endl;

    // Compare if they are equal
    if(pose1.equals(pose2, 0.001)) {
        std::cout << "The poses are the same!" << std::endl;
    } else {
        std::cout << "The poses are not the same." << std::endl;
    }
}

void transformPointCloud_translation(pcl::PointCloud<PointType>::Ptr cloud, float translation_x, float translation_y, float translation_z) {
    if (cloud->empty()) {
        std::cerr << "Error: Point cloud is empty. Cannot perform transformation." << std::endl;
        return;
    }

    // Define the transformation matrix using Eigen
    Eigen::Affine3f transformation_matrix = Eigen::Affine3f::Identity();

    // Apply translation
    transformation_matrix.translation() << translation_x, translation_y, translation_z;

    // Apply the transformation to the point cloud
    pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);
}


// Function that adjusts the cloud to local coordinates
void translateCloudToLocalCoordinatesTranslationOnly(pcl::PointCloud<PointType>::Ptr keyframeCloud, Session &sess, const int &loop_idx_session)
{
    // Translate the source point cloud to the position of the central point cloud in global coordinate system -> to be able to perform ICP with a well-constructed point cloud -> not as done in VirtualICP after SC loop detection
    float translation_x = sess.cloudKeyPoses6D->points[loop_idx_session].x;
    float translation_y = sess.cloudKeyPoses6D->points[loop_idx_session].y;
    float translation_z = sess.cloudKeyPoses6D->points[loop_idx_session].z;


    // the central scan is moved (with 5 scans from the simulated scans) from the global to the local coord system -> where the central scan is at 0,0,0
    transformPointCloud_translation(std::move(keyframeCloud), -translation_x, -translation_y, -translation_z);

}

/* This function uses the GICP from open3D in a for loop,
 * this way the alignment is more robust and promising in comparison with PCL and NANO-GICP.
// MV In contrast with v2, here a submap from the Source session is created, instead of taking only one single scan
// The exact new line is here ( source_sess.loopFindNearKeyframesCentralCoord(queryKeyframeCloud, loop_idx_source_session, NNKeyframes_for_submap_); //V3
// The idea behind this, is to have more chances to get points in the floor / ceiling, which are very critical for the alignment with the reference map.
  * */
void Slam2ref::addKNNloops()
{
    std::string dir = save_directory_ + "Part6_KNN_Loops/";
    std::filesystem::create_directories(dir);

    std::string dir_1 = dir + "/Step_6_01_Best_Fscore/";
    std::string dir_2 = dir +  "/Step_6_02_second_Best_Fscore/";
    std::filesystem::create_directories(dir_1);
    std::filesystem::create_directories(dir_2);

    // add selected sc loops
    auto &central_sess = sessions_.at(central_sess_idx);
    auto &source_sess = sessions_.at(query_sess_idx);

    auto num_KNNloops = source_sess.nodes_.size();

    std::vector<int> idx_added_loops;
    idx_added_loops.reserve(num_KNNloops);
    int counter_1 = 0;
    int counter_2 = 0;

    // Convert PointCloud<PointTypePose> to PointCloud<pcl::PointXYZ> needed for KNN search
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloudXYZ = convertPointCloud(central_sess.cloudKeyPoses6D);


#pragma omp parallel for num_threads(numberOfCores) default(none) shared( num_KNNloops, central_sess, source_sess, cout, idx_added_loops, counter_1, counter_2, target_cloudXYZ, dir, dir_1 , dir_2)
    for (size_t ith = 0; ith < num_KNNloops; ith++)
    {
        mtx.lock();
        // std::unique_lock<std::mutex> lock(mtx);
        int loop_idx_source_session = ith;

        // Source pose
        PointTypePose* pose_init = &source_sess.cloudKeyPoses6D->points[loop_idx_source_session];

        pcl::PointXYZ pose_init_XYZ = transformPoint(*pose_init);

        // Call the function to find k-nearest points
        std::vector<int> nearestIndices = findKNearestPoints(target_cloudXYZ, pose_init_XYZ, NNKeyframes_for_submap_);
        int loop_idx_central_session = nearestIndices[0]; // nearestPoseIndexInCentralSession


        pcl::PointCloud<PointType>::Ptr centralKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr aligned_pcl(new pcl::PointCloud<PointType>());
        // No lock needed here since no concurrent modifications occur on the central_session
        central_sess.createPointCloudWithIndices(centralKeyframeCloud, nearestIndices, "central");

        std::string paddedIndex = std::to_string(ith + 1);
        paddedIndex = std::string(4 - paddedIndex.length(), '0') + paddedIndex;


        // source point cloud
        pcl::PointCloud<PointType>::Ptr queryKeyframeCloud(new pcl::PointCloud<PointType>());
//        *queryKeyframeCloud += *transformPointCloud(source_sess.cloudKeyFrames[ith], pose_init); // V2
        source_sess.loopFindNearKeyframesCentralCoord(queryKeyframeCloud, loop_idx_source_session, NNKeyframes_for_submap_); //V3

        // NEW in V2:
        // translating both point clouds to the local coord system of the source scan ->
        // to be able to perform YawGICP
        translateCloudToLocalCoordinatesTranslationOnly(queryKeyframeCloud, source_sess, loop_idx_source_session);
        translateCloudToLocalCoordinatesTranslationOnly(centralKeyframeCloud, source_sess, loop_idx_source_session); // Does not work if the translation between session is not compensated as result of the ICP
        auto central_translation_x = central_sess.cloudKeyPoses6D->points[loop_idx_central_session].x;
        auto central_translation_y = central_sess.cloudKeyPoses6D->points[loop_idx_central_session].y;
        auto central_translation_z = central_sess.cloudKeyPoses6D->points[loop_idx_central_session].z;

        auto source_translation_x = source_sess.cloudKeyPoses6D->points[loop_idx_source_session].x;
        auto source_translation_y = source_sess.cloudKeyPoses6D->points[loop_idx_source_session].y;
        auto source_translation_z = source_sess.cloudKeyPoses6D->points[loop_idx_source_session].z;

        float compensation_translation_x = source_translation_x - central_translation_x;
        float compensation_translation_y = source_translation_y - central_translation_y;
        float compensation_translation_z = source_translation_z - central_translation_z;
        // the next is only for debugging
        //        if(ith == 0)
        //        {
        //            // writing to a file
        //            std::ofstream outfile(dir + "translations.txt");
        //            outfile << "central_translation_x: " << central_translation_x << std::endl;
        //            outfile << "central_translation_y: " << central_translation_y << std::endl;
        //            outfile << "central_translation_z: " << central_translation_z << std::endl;
        //            outfile << "source_translation_x: " << source_translation_x << std::endl;
        //            outfile << "source_translation_y: " << source_translation_y << std::endl;
        //            outfile << "source_translation_z: " << source_translation_z << std::endl;
        //            outfile << "compensation_translation_x: " << compensation_translation_x << std::endl;
        //            outfile << "compensation_translation_y: " << compensation_translation_y << std::endl;
        //            outfile << "compensation_translation_z: " << compensation_translation_z;
        //            outfile.close();
        //        }



        if(params->save_all_scans_of_KNN_loops_)
        {
            // lock.lock();
            // No lock needed since each file is unique to a thread
            pcl::io::savePCDFile(dir + paddedIndex + "_source_initial_scan_after_1st_T.pcd", *queryKeyframeCloud);
            pcl::io::savePCDFile(dir + paddedIndex + "_target_scan.pcd", *centralKeyframeCloud);
            // lock.unlock();
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


        auto[result_matrix, fitness] = PointCloudRegistration::PerformYawGICP(
                open3d_queryKeyframeCloud, open3d_centralKeyframeCloud, params->max_dist_thr_KNN_ICP_, params->max_dist_thr_KNN_ICP_2_, params->KNN_ICP_version_);


        std::ostringstream stream;
        stream << "Scan: "<< paddedIndex << "; GICP fitness score: " << std::fixed << std::setprecision(3) << fitness << std::endl;
        std::cout << stream.str();

        // Part 6: Get pose transformation and return the relative pose
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame = static_cast<Affine3f>(result_matrix.cast<float>());

        float yaw_init = pose_init->yaw;
        float pitch_init = pose_init->pitch;
        float roll_init = pose_init->roll;

        // pass the sums to the function
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        // calculate the intermediate sums first
        x = x + compensation_translation_x;
        y = y + compensation_translation_y;
        z = z + compensation_translation_z;

        gtsam::Pose3 poseInit2 = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll_init, pitch_init, yaw_init), gtsam::Point3(0, 0, 0));
        gtsam::Pose3 gicp_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        gtsam::Pose3 poseFrom = gicp_pose.compose(poseInit2); // The initial rotation has
        // to be added since it was considered as given before ICP ->
        // the source scanned was rotated to the current calculated pose
        // so that it matches better with the target scan
        gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));


        std::optional<gtsam::Pose3> relative_pose_optional;
        if(use_original_WRONG_ICP_KNN_result_) // left to allow experiments
        {
            relative_pose_optional = poseFrom.between(poseTo); // MV: this is Wrong
        }
        else{
        relative_pose_optional = poseTo.between(poseFrom); // MV: this is Correct
        }

        bool passed = fitness > loopFitnessScoreThreshold_Open3D_KNN_;

        if (passed) {
            auto noiseModel =  (params->noise_model_for_best_KNN_loops_ == "odom") ? odomNoise : robustNoise;
            addBetweenFactorWithAnchoring(loop_idx_central_session, loop_idx_source_session,relative_pose_optional, noiseModel);
            counter_1++;

            if(params->save_all_scans_of_KNN_loops_) {
                *aligned = *open3d_queryKeyframeCloud;
                aligned->Transform(result_matrix);

                colorOpen3DPointCloud(*aligned, blue_color_utils);
                io::WritePointCloud(dir_1 + paddedIndex + "_FINAL_source_cloud_after_KNN_Loop.pcd", *aligned);
            }

        }
        else if (fitness > loopFitnessScoreThreshold_Open3D_KNN_2){
            auto noiseModel =  (params->noise_model_for_second_best_KNN_loops_ == "odom") ? odomNoise : robustNoise;
            addBetweenFactorWithAnchoring(loop_idx_central_session, loop_idx_source_session, relative_pose_optional, noiseModel);
            counter_2++;

            if(params->save_all_scans_of_KNN_loops_) {
                *aligned = *open3d_queryKeyframeCloud;
                aligned->Transform(result_matrix);

                colorOpen3DPointCloud(*aligned, blue_color_utils);
                io::WritePointCloud(dir_2 + paddedIndex + "_FINAL_source_cloud_after_KNN_Loop.pcd", *aligned);
            }

        }
        else{
            if(params->save_all_scans_of_KNN_loops_) {
                *aligned = *open3d_queryKeyframeCloud;
                aligned->Transform(result_matrix);

                colorOpen3DPointCloud(*aligned, blue_color_utils);
                io::WritePointCloud(dir + paddedIndex + "_FINAL_source_cloud_after_KNN_Loop.pcd", *aligned);
            }

        }
    }

    // Part 5: Output the total number of inter-session loops found
    logInfo_cout("\033[1;32m Total ", counter_1 + counter_2 , " (KNN loops) inter-session loops passed the ICP thresholds and were added to the graph. " , counter_1 ," passed the first thr, ", counter_2 ," passed the second thr \033[0m");

} // addKNNloops



