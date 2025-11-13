// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#pragma once

//#include <ros/node_handle.h>
//
//class RosNodeHandle
//{
//public:
//    ros::NodeHandle nh_super;
//}; // class: RosNodeHandle
//
////MV: The following could be used/extended if there is a need of ROS integration, maybe for real-time applications.
//class RosParamServer: public RosNodeHandle
//{
//
//public:
//    ros::NodeHandle & nh;
//
//    std::string sessions_dir_;
//    std::string central_sess_name_;
//    std::string query_sess_name_;
//
//    std::string save_directory_;
//    std::string configPath_;
//
//    bool is_display_debug_msgs_;
//    bool display_debug_msgs_RS_loops_;
//    bool do_not_edit_central_session_;
//    bool use_original_lt_slam_code_;
//
//    int historyKeyframeSearchNum;
//    int NNKeyframes_for_submap_;
//    int MV_ICP_type_in_SC_;
//
//    int kNumSCLoopsUpperBound;
//    int kNumRSLoopsUpperBound;
//    float kICPFilterSize;
//
//    int numberOfCores_;
//
//    float loopFitnessScoreThreshold;
//    float loopFitnessScoreThreshold_Open3D_;
//    float loopFitnessScoreThreshold_Open3D_KNN_;
//    float loopFitnessScoreThreshold_Open3D_KNN_2;
//
//    std::string mEstimator_;
//
//    float robust_noise_model_value_;
//
//    double SC_DIST_THRES_;
//    double SEARCH_RATIO_;
//    double PC_MAX_RADIUS_;
//    double max_dist_thr_original_SC_ICP_;
//    double max_dist_thr_KNN_ICP_;
//    double max_dist_thr_Final_ICP_;
//    int INDOOR_SC_Z_COUNTER_THRES_;
//    int INDOOR_SC_VERSION_;
//    int NUM_CANDIDATES_FROM_TREE_;
//    int PC_NUM_SECTOR_;
//    int PC_NUM_RING_;
//    bool use_indoor_scan_context_;
//    bool filter_scan_pc_with_radius_;
//    bool filter_scan_pc_with_vertical_projection_;
//
//    bool using_MV_modified_version_of_ICP;
//    bool using_MV_performing_final_ICP;
//    bool load_only_scans_and_kitti_files;
//    bool using_MV_performing_KNN_loops;
//
//    bool print_config_param;
//    bool print_ISAM_config_param;
//    bool load_yaml_without_ros_;
//    bool write_merged_central_point_cloud_;
//    bool write_merged_query_point_cloud_;
//    bool use_original_WRONG_ICP_SC_result_;
//    bool use_original_WRONG_ICP_RS_result_;
//    bool use_original_WRONG_ICP_KNN_result_;
//
//    bool MV_skipp_RS_loopDetection_;
//    bool print_SC_ICP_test_values_for_each_iteration_;
//
//
//public:
//    RosParamServer();
//    void load_params(const std::string& configPath);
//
//}; // class: RosParamServer
