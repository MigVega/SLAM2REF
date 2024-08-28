//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//


#ifndef slam2ref_PARAMSERVER_H
#define slam2ref_PARAMSERVER_H

#include <iostream>
#include <memory>
#include <fstream>
#include <chrono>
#include <string>
#include "yaml-cpp/yaml.h"

class ParamServer {
public:
    // Instance method for Singleton
    static ParamServer& getInstance() {
        static ParamServer singleInstance;
        return singleInstance;
    }

    // Delete copy constructor
    ParamServer(ParamServer const&) = delete;

    // Delete assignment operator
    void operator=(ParamServer const&) = delete;


    std::string sessions_dir_;
    std::string central_sess_name_;
    std::string query_sess_name_;

    std::string save_directory_;
    std::string new_folder_test_name_;
    std::string configPath_;
    std::string path_to_target_pc_for_final_ICP_; // This is to the TLS dense PC or from BIM -> used for final ICP and to filter out bad scans

    bool is_display_debug_msgs_;
    bool display_debug_msgs_RS_loops_;
    bool do_not_edit_central_session_;
    bool use_original_lt_slam_code_; // if this is TRUE,
    // a very similar version of the original ltslam algorithm will be loaded
    // (some parameters/thresholds might be different)

    int historyKeyframeSearchNum;
    int NNKeyframes_for_submap_;
    int MV_ICP_type_in_SC_;

    int kNumSCLoopsUpperBound;
    int kNumRSLoopsUpperBound;
    float kICPFilterSize;

    int numberOfCores_;

    float loopFitnessScoreThreshold;
    float loopFitnessScoreThreshold_Open3D_;
    float loopFitnessScoreThreshold_Open3D_KNN_;
    float loopFitnessScoreThreshold_Open3D_KNN_2;
    float loopFitnessScoreThreshold_Final_ICP_;
    float loopFitnessScoreThreshold_Final_ICP_2_;
    float range_distance_for_second_fitness_;
    float loopFitnessScoreThreshold_Final_ICP_3cm_;
    float loopFitnessScoreThreshold_Final_ICP_3cm_2_;
    float loopRmseThreshold_Final_ICP_;
    float loopRmseThreshold_Final_ICP_2_;

    std::string mEstimator_;
    std::string noise_model_for_best_KNN_loops_;
    std::string noise_model_for_second_best_KNN_loops_;

    float robust_noise_model_value_;

    double SC_DIST_THRES_;
    double SEARCH_RATIO_;
    double PC_MAX_RADIUS_;
    double max_dist_thr_original_SC_ICP_;
    double max_dist_thr_SC_ICP_;
    double max_dist_thr_SC_ICP_2_;
    double max_dist_thr_KNN_ICP_;
    double max_dist_thr_KNN_ICP_2_;
    double max_dist_thr_Final_ICP_;
    double max_dist_thr_Final_ICP_2;
    double max_dist_thr_Final_ICP_3;
    int KNN_ICP_version_;
    int INDOOR_SC_Z_COUNTER_THRES_;
    int INDOOR_SC_VERSION_;
    int NUM_CANDIDATES_FROM_TREE_;
    int PC_NUM_SECTOR_;
    int PC_NUM_RING_;
    bool use_indoor_scan_context_;
    bool filter_scan_pc_with_radius_;
    bool filter_scan_pc_with_vertical_projection_;

    bool using_MV_modified_version_of_ICP;
    bool using_MV_performing_final_ICP;
    bool load_only_scans_and_kitti_files;
    bool using_MV_performing_KNN_loops;

    bool print_config_param;
    bool print_ISAM_config_param;
    bool load_yaml_without_ros_;
    bool write_merged_central_point_cloud_;
    bool write_merged_query_point_cloud_;
    bool use_original_WRONG_ICP_SC_result_;
    bool use_original_WRONG_ICP_RS_result_;
    bool use_original_WRONG_ICP_KNN_result_;

    bool MV_skipp_RS_loopDetection_;
    bool print_SC_ICP_test_values_for_each_iteration_;
    bool save_all_scans_of_KNN_loops_;
    bool save_all_source_and_target_scans_in_final_ICP_;
    bool save_registered_source_scans_in_final_ICP_;
    bool write_input_merged_query_session_as_point_cloud_;

    int start_group_index_Final_ICP_; // Only for debugging -> otherwise 0
private:
    ParamServer() {
//        load_params("../../../../config/params.yaml");
        load_params("../config/params.yaml");
    }
    void load_params(const std::string& configPath);
};


#endif //slam2ref_PARAMSERVER_H
