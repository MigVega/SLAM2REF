//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "0.Utils/ParamServer.h"
#include <filesystem>
#include "yaml-cpp/yaml.h"
#include <string>
#include <chrono>
#include <thread>


void ParamServer::load_params(const std::string& configPath)
{
    std::cout << "  Loading params from YAML file" << std::endl;

    // Check if the file exists and can be opened
    std::ifstream file(configPath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open the YAML file at path: " << configPath << std::endl;
        std::cerr << "Please check if the file exists, the path is correct, and you have read permissions." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    file.close();

    try {

        YAML::Node config = YAML::LoadFile(configPath);
        configPath_ = configPath;

        sessions_dir_ = config["slam2ref"]["sessions_dir"].as<std::string>();
        central_sess_name_ = config["slam2ref"]["central_sess_name"].as<std::string>();
        query_sess_name_ = config["slam2ref"]["query_sess_name"].as<std::string>();
        new_folder_test_name_ = config["slam2ref"]["new_folder_test_name"].as<std::string>();
        save_directory_ = config["slam2ref"]["save_directory"].as<std::string>();
        save_directory_ = save_directory_ + new_folder_test_name_;

        path_to_target_pc_for_final_ICP_ = config["slam2ref"]["path_to_target_pc_for_final_ICP"].as<std::string>();

        use_original_lt_slam_code_ = config["slam2ref"]["use_original_lt_slam_code"].as<bool>();

        if(use_original_lt_slam_code_)
        {
            using_MV_modified_version_of_ICP = false;
            using_MV_performing_final_ICP = false;
            using_MV_performing_KNN_loops = false;
            MV_ICP_type_in_SC_ = 0;

            use_original_WRONG_ICP_SC_result_ = true;
            use_original_WRONG_ICP_RS_result_ = true;
            use_original_WRONG_ICP_KNN_result_ = true;
            robust_noise_model_value_ = 0.5;
            mEstimator_ = "Cauchy";
//            NUM_CANDIDATES_FROM_TREE_ = 3; // This makes a big difference, therefore, it is read from YAML anyway
            MV_skipp_RS_loopDetection_ = false;
        }
        else{
            using_MV_modified_version_of_ICP = config["slam2ref"]["using_MV_modified_version_of_ICP"].as<bool>(); //Best: true
            using_MV_performing_final_ICP = config["slam2ref"]["using_MV_performing_final_ICP"].as<bool>(); //Best: true
            using_MV_performing_KNN_loops = config["slam2ref"]["using_MV_performing_KNN_loops"].as<bool>(); //Best: true
            use_original_WRONG_ICP_SC_result_ = config["slam2ref"]["use_original_WRONG_ICP_SC_result"].as<bool>(); //Best: false
            use_original_WRONG_ICP_RS_result_ = config["slam2ref"]["use_original_WRONG_ICP_RS_result"].as<bool>(); //Best: false
            use_original_WRONG_ICP_KNN_result_ = config["slam2ref"]["use_original_WRONG_ICP_KNN_result"].as<bool>(); //Best: false
            MV_ICP_type_in_SC_ = config["slam2ref"]["MV_ICP_type_in_SC"].as<int>(); //Best: true

            MV_skipp_RS_loopDetection_ = config["slam2ref"]["MV_skipp_RS_loopDetection"].as<bool>();

            // NOT that relevant
            robust_noise_model_value_ = config["slam2ref"]["robust_noise_model_value"].as<float>();
            mEstimator_ = config["slam2ref"]["mEstimator"].as<std::string>();
        }
        NUM_CANDIDATES_FROM_TREE_ = config["scan_context"]["NUM_CANDIDATES_FROM_TREE"].as<int>();
        start_group_index_Final_ICP_ = config["slam2ref"]["start_group_index_Final_ICP"].as<int>();
        noise_model_for_best_KNN_loops_ = config["slam2ref"]["noise_model_for_best_KNN_loops"].as<std::string>();
        noise_model_for_second_best_KNN_loops_ = config["slam2ref"]["noise_model_for_second_best_KNN_loops"].as<std::string>();

        do_not_edit_central_session_ = config["slam2ref"]["do_not_edit_central_session"].as<bool>();
        max_dist_thr_original_SC_ICP_ = config["slam2ref"]["max_dist_thr_original_SC_ICP"].as<double>();

        max_dist_thr_SC_ICP_ = config["slam2ref"]["max_dist_thr_SC_ICP"].as<double>();
        max_dist_thr_SC_ICP_2_ = config["slam2ref"]["max_dist_thr_SC_ICP_2"].as<double>();

        KNN_ICP_version_ = config["slam2ref"]["KNN_ICP_version"].as<int>();
        max_dist_thr_KNN_ICP_ = config["slam2ref"]["max_dist_thr_KNN_ICP"].as<double>();
        max_dist_thr_KNN_ICP_2_ = config["slam2ref"]["max_dist_thr_KNN_ICP_2"].as<double>();
        max_dist_thr_Final_ICP_ = config["slam2ref"]["max_dist_thr_Final_ICP"].as<double>();
        max_dist_thr_Final_ICP_2 = config["slam2ref"]["max_dist_thr_Final_ICP_2"].as<double>();
        max_dist_thr_Final_ICP_3 = config["slam2ref"]["max_dist_thr_Final_ICP_3"].as<double>();

        use_indoor_scan_context_ = config["scan_context"]["use_indoor_scan_context"].as<bool>();
        INDOOR_SC_VERSION_ = config["scan_context"]["INDOOR_SC_VERSION"].as<int>();
        INDOOR_SC_Z_COUNTER_THRES_ = config["scan_context"]["INDOOR_SC_Z_COUNTER_THRES"].as<int>();

        SC_DIST_THRES_ = config["scan_context"]["SC_DIST_THRES"].as<double>();
        SEARCH_RATIO_ = config["scan_context"]["SEARCH_RATIO"].as<double>();
        PC_MAX_RADIUS_ = config["scan_context"]["PC_MAX_RADIUS"].as<double>();


        filter_scan_pc_with_radius_ = config["scan_context"]["filter_scan_pc_with_radius"].as<bool>();
        filter_scan_pc_with_vertical_projection_ = config["scan_context"]["filter_scan_pc_with_vertical_projection"].as<bool>();


        PC_NUM_RING_ = config["scan_context"]["PC_NUM_RING"].as<int>();
        PC_NUM_SECTOR_ = config["scan_context"]["PC_NUM_SECTOR"].as<int>();

        load_only_scans_and_kitti_files = config["slam2ref"]["load_only_scans_and_kitti_files"].as<bool>();


        historyKeyframeSearchNum = config["slam2ref"]["historyKeyframeSearchNum"].as<int>();
        NNKeyframes_for_submap_ = config["slam2ref"]["NNKeyframes_for_submap"].as<int>();


        kNumSCLoopsUpperBound = config["slam2ref"]["kNumSCLoopsUpperBound"].as<int>();
        kNumRSLoopsUpperBound = config["slam2ref"]["kNumRSLoopsUpperBound"].as<int>();
        kICPFilterSize = config["slam2ref"]["kICPFilterSize"].as<float>();

        loopFitnessScoreThreshold = config["slam2ref"]["loopFitnessScoreThreshold"].as<float>();
        loopFitnessScoreThreshold_Open3D_ = config["slam2ref"]["loopFitnessScoreThreshold_Open3D"].as<float>();
        loopFitnessScoreThreshold_Open3D_KNN_ = config["slam2ref"]["loopFitnessScoreThreshold_Open3D_KNN"].as<float>();
        loopFitnessScoreThreshold_Open3D_KNN_2 = config["slam2ref"]["loopFitnessScoreThreshold_Open3D_KNN_2"].as<float>();
        loopFitnessScoreThreshold_Final_ICP_ = config["slam2ref"]["loopFitnessScoreThreshold_Final_ICP"].as<float>();
        loopFitnessScoreThreshold_Final_ICP_2_ = config["slam2ref"]["loopFitnessScoreThreshold_Final_ICP_2"].as<float>();

        range_distance_for_second_fitness_ = config["slam2ref"]["range_distance_for_second_fitness"].as<float>();
        loopFitnessScoreThreshold_Final_ICP_3cm_ = config["slam2ref"]["loopFitnessScoreThreshold_Final_ICP_3cm"].as<float>();
        loopFitnessScoreThreshold_Final_ICP_3cm_2_ = config["slam2ref"]["loopFitnessScoreThreshold_Final_ICP_3cm_2"].as<float>();
        loopRmseThreshold_Final_ICP_ = config["slam2ref"]["loopRmseThreshold_Final_ICP"].as<float>();
        loopRmseThreshold_Final_ICP_2_ = config["slam2ref"]["loopRmseThreshold_Final_ICP_2"].as<float>();

        // GENERAL (NOT RELEVANT) ----------------------------------------------
        numberOfCores_ = config["slam2ref"]["numberOfCores"].as<int>();

        print_config_param = config["slam2ref"]["print_config_param"].as<bool>();
        print_ISAM_config_param = config["slam2ref"]["print_ISAM_config_param"].as<bool>();

        load_yaml_without_ros_= config["slam2ref"]["load_yaml_without_ros"].as<bool>();
        write_merged_central_point_cloud_= config["slam2ref"]["write_merged_central_point_cloud"].as<bool>();
        write_merged_query_point_cloud_= config["slam2ref"]["write_merged_query_point_cloud"].as<bool>();
        write_input_merged_query_session_as_point_cloud_= config["slam2ref"]["write_input_merged_query_session_as_point_cloud"].as<bool>();
        save_all_scans_of_KNN_loops_= config["slam2ref"]["save_all_scans_of_KNN_loops"].as<bool>();
        save_all_source_and_target_scans_in_final_ICP_= config["slam2ref"]["save_all_source_and_target_scans_in_final_ICP_"].as<bool>();
        save_registered_source_scans_in_final_ICP_= config["slam2ref"]["save_registered_source_scans_in_final_ICP"].as<bool>();

        is_display_debug_msgs_ = config["slam2ref"]["is_display_debug_msgs"].as<bool>();
        display_debug_msgs_RS_loops_ = config["slam2ref"]["display_debug_msgs_RS_loops"].as<bool>();
        print_SC_ICP_test_values_for_each_iteration_ = config["slam2ref"]["print_SC_ICP_test_values_for_each_iteration"].as<bool>();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    catch(const YAML::ParserException& e) {
        std::cout << "Caught a bad conversion: " << e.what() << std::endl;
        std::cout << "Error occurred at Line: " << e.mark.line+1;
        std::cout << ", Column: " << e.mark.column+1 << std::endl;
        std::exit(EXIT_FAILURE);
    }
    catch (const YAML::BadConversion& e) {
        std::cout << "Caught a bad conversion of the input param.yaml: " << e.what() << std::endl;
        std::cout << "Error occurred at Line: " << e.mark.line+1;
        std::cout << ", Column: " << e.mark.column+1 << std::endl;
        std::exit(EXIT_FAILURE);
    }
    catch (const YAML::BadFile& e) {
        std::cerr << "Failed to open the YAML file: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    } catch (const YAML::BadDereference& e) {
        std::cerr << "Dereferencing a null or invalid YAML node: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    } catch (const YAML::BadSubscript& e) {
        std::cerr << "Subscript operation failed on a YAML node: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    } catch (const std::exception& e) {
        std::cerr << "An unexpected standard exception occurred: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    } catch (...) {
        std::cerr << "An unexpected exception occurred." << std::endl;
        std::exit(EXIT_FAILURE);
    }
}