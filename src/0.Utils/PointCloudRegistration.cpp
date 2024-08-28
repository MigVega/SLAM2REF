//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "0.Utils/PointCloudRegistration.h"
using namespace open3d;
using namespace std;
using namespace Eigen;


// This function calculates covariances of the source point cloud
// Now it is based on Point2Point ICP (therefore, it does not require covariances
// -> and is not use For SC loop detection
std::tuple<Eigen::Matrix4d, double, double> PointCloudRegistration::make_icp_v8_p2p(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Matrix4d &initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration, bool downsampleSource)
{
    std::shared_ptr<open3d::geometry::PointCloud> source = std::make_shared<open3d::geometry::PointCloud>();
    if(downsampleSource){
        auto voxel_size = 0.02; //without this, some scans will not be correclty registered
        source = source_cloud.VoxelDownSample(voxel_size);
    }
    else{
        source = std::make_shared<open3d::geometry::PointCloud>(source_cloud);
    }

    Eigen::Matrix<double, 4, 4> transformation_matrix = initial_guess;

    std::shared_ptr<visualization::Visualizer> vis;
    if(visual) {
        auto visual_cloud = std::make_shared<geometry::PointCloud>(visual_target_cloud);
        cout << "\n   ----------- Making Open3D P2P ICP V7 ----------\n\n";
        vis = std::make_shared<visualization::Visualizer>();
        vis->CreateVisualizerWindow();
        vis->AddGeometry(visual_cloud);
        vis->AddGeometry(source);
    }

    Matrix4d iteration_transformation = Matrix4d::Identity();
    source->Transform(transformation_matrix);


    open3d::pipelines::registration::RegistrationResult icp;

    for (int iteration = 0; iteration < 1; ++iteration) {

        // Perform ICP registration
        icp = open3d::pipelines::registration::RegistrationICP(
                *source, target_cloud, thr_max_dist,
                Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPoint(), //  Here it works better than GICP!
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-7, 1e-7, iterations)
        );

        // Get the transformation matrix from the ICP result
        iteration_transformation = icp.transformation_ * iteration_transformation;
        if(print_after_each_iteration) {
            cout << "\n       ----------- in ICP for loop------------\n";
            cout << "Iteration " << iteration << "\n";
            cout << "fitness: " << icp.fitness_ << "\n";
            cout << " inlier_rmse: " << icp.inlier_rmse_ << "\n";
        }

        // Apply the transformation to the source point cloud for visualization
        source->Transform(icp.transformation_);

        if(visual) {
            // Update the visualization
            vis->UpdateGeometry(source);
            vis->PollEvents();
            vis->UpdateRender();
        }

    }

    if(visual) {
        vis->DestroyVisualizerWindow();
    }
    auto fitness = icp.fitness_;
    auto inlier_rmse = icp.inlier_rmse_;

    //Return the result
    transformation_matrix = iteration_transformation * transformation_matrix;

    return std::make_tuple(transformation_matrix, fitness, inlier_rmse);
}

// This function calculates covariances of the source point cloud
// Now It is based on Point2Point ICP (therefore, it does not require covariances)
std::tuple<Eigen::Matrix4d, double, double> PointCloudRegistration::make_icp_v7(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Matrix4d &initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration, const double voxel_size,const double radius,const int max_nn)
{
    auto source = source_cloud.VoxelDownSample(voxel_size);

    auto source_tree = std::make_shared<geometry::KDTreeFlann>(*source);
    source->EstimateCovariances(geometry::KDTreeSearchParamHybrid(radius,max_nn));

    Eigen::Matrix<double, 4, 4> transformation_matrix = initial_guess;

    std::shared_ptr<visualization::Visualizer> vis;
    if(visual) {
        auto visual_cloud = std::make_shared<geometry::PointCloud>(visual_target_cloud);
        cout << "\n   ----------- Making Open3D P2P ICP V7 ----------\n\n";
        vis = std::make_shared<visualization::Visualizer>();
        vis->CreateVisualizerWindow();
        vis->AddGeometry(visual_cloud);
        vis->AddGeometry(source);
    }

    Matrix4d iteration_transformation = Matrix4d::Identity();
    source->Transform(transformation_matrix);

    std::array<double, 3> last_three_fitnesses = {0.0, 0.0, 0.0}; // Variable to store the last three fitness scores

    open3d::pipelines::registration::RegistrationResult icp;

    for (int iteration = 0; iteration < 1; ++iteration) {

        // Perform ICP registration
        icp = open3d::pipelines::registration::RegistrationICP(
                *source, target_cloud, thr_max_dist,
                Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPoint(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-7, 1e-7, iterations)
        );

        // Get the transformation matrix from the ICP result
        iteration_transformation = icp.transformation_ * iteration_transformation;
        if(print_after_each_iteration) {
            cout << "\n       ----------- in ICP for loop------------\n";
            cout << "Iteration " << iteration << "\n";
            cout << "fitness: " << icp.fitness_ << "\n";
            cout << " inlier_rmse: " << icp.inlier_rmse_ << "\n";
        }

        // Apply the transformation to the source point cloud for visualization
        source->Transform(icp.transformation_);

        if(visual) {
            // Update the visualization
            vis->UpdateGeometry(source);
            vis->PollEvents();
            vis->UpdateRender();
        }
        // Shift the last three fitness scores and add the new one (rounded to 3 decimal places)
        last_three_fitnesses = {last_three_fitnesses[1],
                                last_three_fitnesses[2],
                                std::round(icp.fitness_ * 1000) / 1000 };

        // Check if the last three fitness scores are the same and, if so, break the loop
        if ((last_three_fitnesses[0] == last_three_fitnesses[1] && last_three_fitnesses[1] == last_three_fitnesses[2]) || icp.fitness_ > 0.98) {
            break;
        }
    }

    if(visual) {
        vis->DestroyVisualizerWindow();
    }
    auto fitness = icp.fitness_;
    auto inlier_rmse = icp.inlier_rmse_;

    //Return the result
    transformation_matrix = iteration_transformation * transformation_matrix;

    return std::make_tuple(transformation_matrix, fitness, inlier_rmse);
}


std::tuple<Eigen::Matrix4d, double, double> PointCloudRegistration::make_icp_v5_open3D(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Matrix4d &initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration)
{
    auto voxel_size = 0.02; // 2 cm TODO move this to the params in yaml
    auto source = source_cloud.VoxelDownSample(voxel_size);
    auto target = target_cloud.VoxelDownSample(voxel_size);
//    auto transformation_matrix = initial_guess;
    Eigen::Matrix<double, 4, 4> transformation_matrix = initial_guess;

    std::shared_ptr<visualization::Visualizer> vis;
    if(visual) {
        auto visual_cloud = std::make_shared<geometry::PointCloud>(visual_target_cloud);
        cout << "\n   ----------- Making Open3D GICP V5 ----------\n\n";
        vis = std::make_shared<visualization::Visualizer>();
        vis->CreateVisualizerWindow();
        vis->AddGeometry(visual_cloud);
        vis->AddGeometry(source);
    }

    Matrix4d iteration_transformation = Matrix4d::Identity();
    source->Transform(transformation_matrix);

    std::array<double, 3> last_three_fitnesses = {0.0, 0.0, 0.0}; // Variable to store the last three fitness scores

    open3d::pipelines::registration::RegistrationResult icp;

    for (int iteration = 0; iteration < iterations; ++iteration) {

        // Perform ICP registration
        icp = open3d::pipelines::registration::RegistrationICP(
                *source, *target, thr_max_dist,
                Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 1)
        );

        // Get the transformation matrix from the ICP result
        iteration_transformation = icp.transformation_ * iteration_transformation;
        if(print_after_each_iteration) {
            cout << "\n       ----------- in ICP for loop------------\n";
            cout << "Iteration " << iteration << "\n";
            cout << "fitness: " << icp.fitness_ << "\n";
            cout << " inlier_rmse: " << icp.inlier_rmse_ << "\n";
        }

        // Apply the transformation to the source point cloud for visualization
        source->Transform(icp.transformation_);

        if(visual) {
            // Update the visualization
            vis->UpdateGeometry(source);
            vis->PollEvents();
            vis->UpdateRender();
        }
        // Shift the last three fitness scores and add the new one (rounded to 3 decimal places)
        last_three_fitnesses = {last_three_fitnesses[1],
                                last_three_fitnesses[2],
                                std::round(icp.fitness_ * 1000) / 1000 };

        // Check if the last three fitness scores are the same and, if so, break the loop
        if (last_three_fitnesses[0] == last_three_fitnesses[1] && last_three_fitnesses[1] == last_three_fitnesses[2]) {
            break;
        }
    }

    if(visual) {
        vis->DestroyVisualizerWindow();
    }
    auto fitness = icp.fitness_;
    auto inlier_rmse = icp.inlier_rmse_;

    //Return the result
    transformation_matrix = iteration_transformation * transformation_matrix;

    return std::make_tuple(transformation_matrix, fitness, inlier_rmse);
}


std::tuple<Eigen::Matrix4d, double, double>  PointCloudRegistration::make_icp_v6_only_yaw(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Matrix4d& initial_guess, int iterations, double thr_max_dist, bool visual, bool print_after_each_iteration)
{
    auto voxel_size = 0.02;
    auto source = source_cloud.VoxelDownSample(voxel_size);
    auto target = target_cloud.VoxelDownSample(voxel_size);

    Eigen::Matrix<double, 4, 4> transformation_matrix = initial_guess;

    std::shared_ptr<visualization::Visualizer> vis;
    if(visual) {
        auto visual_cloud = std::make_shared<geometry::PointCloud>(visual_target_cloud);
//        cout << "\n   ----------- Making Open3D GICP V5 ----------\n\n";
        vis = std::make_shared<visualization::Visualizer>();
        vis->CreateVisualizerWindow();
        vis->AddGeometry(visual_cloud);
        vis->AddGeometry(source);
    }

    Matrix4d iteration_transformation = Matrix4d::Identity();
    source->Transform(transformation_matrix);

    std::array<double, 3> last_three_fitnesses = {0.0, 0.0, 0.0};

    open3d::pipelines::registration::RegistrationResult icp;

    for (int iteration = 0; iteration < iterations; ++iteration) {

        // Perform ICP registration
        icp = open3d::pipelines::registration::RegistrationICP(
                *source, *target, thr_max_dist,
                Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 1)
        );

        // Get the transformation matrix from the ICP result and extract yaw rotation
        Eigen::Matrix4d icp_transformation = icp.transformation_;
        double yaw = atan2(icp_transformation(1, 0), icp_transformation(0, 0));

        // Create a new transformation matrix that only considers yaw rotation
        Eigen::Matrix4d yaw_only_transformation;
        yaw_only_transformation << cos(yaw), -sin(yaw), 0, icp_transformation(0, 3),
                sin(yaw),  cos(yaw), 0, icp_transformation(1, 3),
                0,        0, 1, icp_transformation(2, 3),
                0,        0, 0, icp_transformation(3, 3);
//      ONLY ROTATION!!! :
//       yaw_only_transformation << cos(yaw), -sin(yaw), 0, 0,
//                sin(yaw),  cos(yaw), 0, 0,
//                0,        0, 1, 0,
//                0,        0, 0, 1;
        // Update the iteration_transform with the yaw_only_transform
        iteration_transformation = yaw_only_transformation * iteration_transformation;

        if(print_after_each_iteration) {
            cout << "\n       ----------- in ICP for loop------------\n";
            cout << "Iteration " << iteration << "\n";
            cout << "fitness: " << icp.fitness_ << "\n";
            cout << " inlier_rmse: " << icp.inlier_rmse_ << "\n";
        }

        // Apply the transformation to the source point cloud for visualization
        source->Transform(yaw_only_transformation);

        if(visual) {
            // Update the visualization
            vis->UpdateGeometry(source);
            vis->PollEvents();
            vis->UpdateRender();
        }

        // Shift the last three fitness scores and add the new one (rounded to 3 decimal places)
        last_three_fitnesses = {last_three_fitnesses[1],
                                last_three_fitnesses[2],
                                std::round(icp.fitness_ * 1000) / 1000 };

        // Check if the last three fitness scores are the same and, if so, break the loop
        if (last_three_fitnesses[0] == last_three_fitnesses[1] && last_three_fitnesses[1] == last_three_fitnesses[2]) {
            break;
        }
    }

    if(visual) {
        vis->DestroyVisualizerWindow();
    }

    auto fitness = icp.fitness_;
    auto inlier_rmse = icp.inlier_rmse_;

    // Update the final transformation_matrix with the iteration_transformation
    transformation_matrix = iteration_transformation * transformation_matrix;

    //Return the result
    return std::make_tuple(transformation_matrix, fitness, inlier_rmse);
}

/*
 *  While working with YawGICP It is critical to have the target point cloud in the local coordinate system of the source scan, (where the origin is at 0,0,0) otherwise the process will yield bad results, due to the fact that the resulted matrix is from the 0,0,0, and the method would rotate the point cloud (yaw) from its origin and not from another coordinate (the 0,0,0 of the global coordinate).
 */
std::tuple<Eigen::Matrix4d, double> PointCloudRegistration::PerformYawGICP(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_queryKeyframeCloud, const std::shared_ptr<open3d::geometry::PointCloud>& open3d_centralKeyframeCloud, const double thr_1, const double thr_2, const int version) {
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

    if(version == 3) // Not Yaw restricted
    {
        std::tie(result_matrix, fitness, inlier_rmse)  = PointCloudRegistration::make_icp_v5_open3D(*open3d_queryKeyframeCloud,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 100, thr_1, false, false);

        std::shared_ptr<geometry::PointCloud> aligned = std::make_shared<geometry::PointCloud>();
        *aligned = *open3d_queryKeyframeCloud;
        aligned->Transform(result_matrix);
        Eigen::Matrix4d result_matrix2;
        // Here only 10 iterations AND with a max_dist of 0.1 this will adjust the fitness calculation! -> leaving out wrongly registered point cluuds with a fitness < 0.3
        std::tie(result_matrix2, fitness, inlier_rmse) = PointCloudRegistration::make_icp_v5_open3D(*aligned,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 10, thr_2, false, false);

        result_matrix = result_matrix2 * result_matrix;
    }
    else if(version == 4) // focus on Yaw rotation
    {
        std::tie(result_matrix, fitness, inlier_rmse)  = PointCloudRegistration::make_icp_v6_only_yaw(*open3d_queryKeyframeCloud,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 100, thr_1, false, false);

        std::shared_ptr<geometry::PointCloud> aligned = std::make_shared<geometry::PointCloud>();
        *aligned = *open3d_queryKeyframeCloud;
        aligned->Transform(result_matrix);
        Eigen::Matrix4d result_matrix2;
        // Here only 10 iterations AND with a max_dist of 0.1 this will adjust the fitness calculation! -> leaving out wrongly registered point cluuds with a fitness < 0.3
        std::tie(result_matrix2, fitness, inlier_rmse) = PointCloudRegistration::make_icp_v5_open3D(*aligned,*open3d_centralKeyframeCloud,*open3d_centralKeyframeCloud,initial_guess, 10, thr_2, false, false);

        result_matrix = result_matrix2 * result_matrix;
    }



    return  std::make_tuple(result_matrix, fitness);
}


std::pair<double, double> PointCloudRegistration::EvaluateRegistration(const open3d::geometry::PointCloud &source,
                                   const open3d::geometry::PointCloud &target,
                                   double max_correspondence_distance,
                                   const Eigen::Matrix4d &result_transformation,
                                   float range_distance) {
    if (!result_transformation.isIdentity()) {
        auto aligned = source;
        aligned.Transform(result_transformation);
        auto corresp = aligned.ComputePointCloudDistance(target);

        int numCorrespondences = 0;
        int numPointsWithinRange = 0;
        double fitness = 0;
        double fitness_in_range = 0;

        for (auto &dist : corresp) {
            if (dist <= max_correspondence_distance) {
                numCorrespondences++;
            }
            if (dist <= range_distance) {
                numPointsWithinRange++;
            }
        }

        if (numCorrespondences > 0) {
            fitness = numCorrespondences/ (double) source.points_.size();  // % Of inliers from the total number of points in source
            fitness_in_range = static_cast<double>(numCorrespondences)/ static_cast<double>(numPointsWithinRange);  // % Of inliers from the total number of points in source
        }

        return std::make_pair(fitness, fitness_in_range);
    } else {
        std::cout << "The transformation is identity, returning infinity.\n";
        return std::make_pair(0, 0);
    }
}


std::pair<double, double> PointCloudRegistration::EvaluateRegistrationAlignedSource(open3d::geometry::PointCloud &alignedSource,   const open3d::geometry::PointCloud &target,  double max_correspondence_distance, float range_distance) {
   auto corresp = alignedSource.ComputePointCloudDistance(target);

   int numCorrespondences = 0;
   int numPointsWithinRange = 0;
   double fitness = 0;
   double fitness_in_range = 0;

   for (auto &dist : corresp) {
       if (dist <= max_correspondence_distance) {
           numCorrespondences++;
       }
       if (dist <= range_distance) {
           numPointsWithinRange++;
       }
   }

   if (numCorrespondences > 0) {
       fitness = numCorrespondences/ (double) alignedSource.points_.size();  // % Of inliers from the total number of points in source
       fitness_in_range = static_cast<double>(numCorrespondences)/ static_cast<double>(numPointsWithinRange);  // % Of inliers from the total number of points in source
   }

   return std::make_pair(fitness, fitness_in_range);

}


std::tuple<double, double, double, double> PointCloudRegistration::EvaluateRegistrationAlignedSourceAndRMSE(
        open3d::geometry::PointCloud& alignedSource,
        const open3d::geometry::PointCloud& target,
        double max_correspondence_distance,
        float range_distance)
{
    auto corresp = alignedSource.ComputePointCloudDistance(target);

    auto sourceSize = static_cast<double>(alignedSource.points_.size());
    int numCorrespondences = 0;
    int numPointsWithinRange = 0;
    double fitness = 0;
    double fitness_in_range = 0;
    double sumSquaredDist = 0; // Sum of squared distances for RMSE
    double sumSquaredDistInRange = 0; // Sum of squared distances in range

    for (auto &dist : corresp) {
        sumSquaredDist += dist * dist; // accumulating sum of squared distances

        if (dist <= max_correspondence_distance) {
            numCorrespondences++;
        }
        if (dist <= range_distance) {
            numPointsWithinRange++;
            sumSquaredDistInRange += dist * dist; // accumulating sum of squared distances in range
        }
    }

    double rmse = std::sqrt(sumSquaredDist / sourceSize); // RMSE for all points
    double rmseInRange = numPointsWithinRange > 0 ? std::sqrt(sumSquaredDistInRange / numPointsWithinRange) : 0; // RMSE for points in range

    if (numCorrespondences > 0) {
        fitness = numCorrespondences / sourceSize;
        fitness_in_range = static_cast<double>(numCorrespondences) / static_cast<double>(numPointsWithinRange);
    }

    return std::make_tuple(fitness, fitness_in_range, rmse, rmseInRange);
}