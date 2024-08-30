//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#pragma once


// GTSAM headers
#include <gtsam/inference/Symbol.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

// Standard headers
#include <memory> // For std::shared_pt

#include <optional>

#include "01.Slam2ref/RosParamServer.h"
#include "0.Utils/ParamServer.h"

#include "01.Slam2ref/BetweenFactorWithAnchoring.h"
#include "01.Slam2ref/Session.h"

using namespace open3d;
using namespace std;
using namespace Eigen;
using namespace gtsam;
using namespace Slam3refParam;

class Slam2ref
{
private:
    ParamServer* params;

    // The following are used for the existing variables -> new variables should be called params->VARIABLE_NAME
    std::string sessions_dir_ = params->sessions_dir_;
    std::string central_sess_name_ = params->central_sess_name_;
    std::string query_sess_name_ = params->query_sess_name_;

    std::string save_directory_ = params->save_directory_;
    std::string configPath_ = params->configPath_;

    bool is_display_debug_msgs_ = params->is_display_debug_msgs_;
    bool display_debug_msgs_RS_loops_ = params->display_debug_msgs_RS_loops_;
    bool do_not_edit_central_session_ = params->do_not_edit_central_session_;
    bool use_original_lt_slam_code_ = params->use_original_lt_slam_code_;

    int historyKeyframeSearchNum = params->historyKeyframeSearchNum;
    int NNKeyframes_for_submap_ = params->NNKeyframes_for_submap_;
    int MV_ICP_type_in_SC_ = params->MV_ICP_type_in_SC_;

    int kNumSCLoopsUpperBound = params->kNumSCLoopsUpperBound;
    int kNumRSLoopsUpperBound = params->kNumRSLoopsUpperBound;
    float kICPFilterSize = params->kICPFilterSize;

    int numberOfCores_ = params->numberOfCores_;

    float loopFitnessScoreThreshold = params->loopFitnessScoreThreshold;
    float loopFitnessScoreThreshold_Open3D_ = params->loopFitnessScoreThreshold_Open3D_;
    float loopFitnessScoreThreshold_Open3D_KNN_ = params->loopFitnessScoreThreshold_Open3D_KNN_;
    float loopFitnessScoreThreshold_Open3D_KNN_2 = params->loopFitnessScoreThreshold_Open3D_KNN_2;

    std::string mEstimator_ = params->mEstimator_;

    float robust_noise_model_value_ = params->robust_noise_model_value_;

    double SC_DIST_THRES_ = params->SC_DIST_THRES_;
    double SEARCH_RATIO_ = params->SEARCH_RATIO_;
    double PC_MAX_RADIUS_ = params->PC_MAX_RADIUS_;
    double max_dist_thr_original_SC_ICP_ = params->max_dist_thr_original_SC_ICP_;
    double max_dist_thr_KNN_ICP_ = params->max_dist_thr_KNN_ICP_;

    double max_dist_thr_Final_ICP_ = params->max_dist_thr_Final_ICP_;
    int INDOOR_SC_Z_COUNTER_THRES_ = params->INDOOR_SC_Z_COUNTER_THRES_;
    int INDOOR_SC_VERSION_ = params->INDOOR_SC_VERSION_;
    int NUM_CANDIDATES_FROM_TREE_ = params->NUM_CANDIDATES_FROM_TREE_;

    int PC_NUM_SECTOR_ = params->PC_NUM_SECTOR_;
    int PC_NUM_RING_ = params->PC_NUM_RING_;

    bool use_indoor_scan_context_ = params->use_indoor_scan_context_;
    bool filter_scan_pc_with_radius_ = params->filter_scan_pc_with_radius_;
    bool filter_scan_pc_with_vertical_projection_ = params->filter_scan_pc_with_vertical_projection_;

    bool using_MV_modified_version_of_ICP = params->using_MV_modified_version_of_ICP;
    bool using_MV_performing_final_ICP = params->using_MV_performing_final_ICP;
    bool load_only_scans_and_kitti_files = params->load_only_scans_and_kitti_files;
    bool using_MV_performing_KNN_loops = params->using_MV_performing_KNN_loops;

    bool print_config_param = params->print_config_param;
    bool print_ISAM_config_param = params->print_ISAM_config_param;
    bool load_yaml_without_ros_ = params->load_yaml_without_ros_;
    bool write_merged_central_point_cloud_ = params->write_merged_central_point_cloud_;
    bool write_merged_query_point_cloud_ = params->write_merged_query_point_cloud_;
    bool use_original_WRONG_ICP_SC_result_ = params->use_original_WRONG_ICP_SC_result_;
    bool use_original_WRONG_ICP_RS_result_ = params->use_original_WRONG_ICP_RS_result_;
    bool use_original_WRONG_ICP_KNN_result_= params->use_original_WRONG_ICP_KNN_result_;

    bool MV_skipp_RS_loopDetection_= params->MV_skipp_RS_loopDetection_;
    bool print_SC_ICP_test_values_for_each_iteration_= params->print_SC_ICP_test_values_for_each_iteration_;
public:
    // Sessions
    Sessions sessions_;
    SessionsDict sessions_dict_;

    // Pose graph 
    
    const int central_sess_idx = 1; // means the central session. recommend using 1 for it (because of the stable indexing for anchor node index)
    const int query_sess_idx = 2;

    std::vector<std::pair<int, int>> SCLoopIdxPairs_;
    std::vector<float> SCLoop_yaw_angles_;
    std::vector<std::pair<int, int>> RSLoopIdxPairs_;

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;

    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    const gtsam::Pose3 poseOrigin;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odomNoise;
    noiseModel::Diagonal::shared_ptr loopNoise;
    noiseModel::Diagonal::shared_ptr largeNoise;
    noiseModel::Base::shared_ptr robustNoise;

    std::mutex mtx;
    const int numberOfCores = numberOfCores_;

    const std::vector<int> green_color = { 0 ,255, 0 }; 
    const std::vector<int> red_color = {255 ,0 , 0 };

    // Define the RGB colors
    Eigen::Vector3i red_color_utils = Eigen::Vector3i(255, 0, 0);
    Eigen::Vector3i black_color_utils = Eigen::Vector3i(0, 0, 0);
    Eigen::Vector3i green_color_utils= Eigen::Vector3i(0, 255, 0);
    Eigen::Vector3i blue_color_utils= Eigen::Vector3i(0, 0, 255);


public:
    Slam2ref();
    ~Slam2ref();

    void run( void );

    //  PART 0 init
    void initNoiseConstants();
    void initOptimizer();

    // PART 1. Load sessions and graph
    /**
     * @brief Loads pose data for all sessions.
     *
     * This function iterates through the sessions directory, loading pose data for central and query sessions.
     * Pose data is read from the specified directory, and nodes, edges, scan context descriptors (SCD), and point cloud (PCD) files are loaded internally.
     *
     * @note The function is currently designed for two-session versions and checks if the session name corresponds to the central or query session.
     * Entries with filenames not matching central_sess_name_ or query_sess_name_ are skipped.
     *
     * @details
     * The function performs the following steps:
     * 1. Iterates through the sessions directory.
     * 2. Checks if the session name is either the central or query session; if not, continues to the next iteration.
     * 3. Determines the session index based on the session name.
     * 4. Constructs the session's directory path.
     * 5. Creates a Session object and inserts it into the sessions_ map.
     * 6. Prints information about loaded sessions, including names and whether they are central sessions.
     * 7. Outputs the total number of loaded sessions.
     *
     * @see Session
     * @see isTwoStringSame
     *
     * @note The total number of loaded sessions is printed with boolean values shown as true or false.
     * @warning The function is currently designed for two-session versions; further generalization for N-session co-optimization is a TODO.
     */
    void loadAllSessions();

    friend int genGlobalNodeIdx(const int&, const int&);
    friend int genAnchorNodeIdx(const int&);

    /**
     * @brief Adds all sessions to the global graph.
     *
     * This function iterates through all sessions and performs the following steps:
     * 1. Initializes the trajectory by anchoring the session.
     * 2. Adds the session to the central graph.
     *
     * @details
     * The function iterates through each session in the sessions_ map and performs the following actions:
     * 1. Initializes the trajectory of the session by anchoring it.
     * 2. Adds the session (all nodes and edges) to the central graph using the addSessionToCentralGraph function.
     *
     * @see Session
     * @see initTrajectoryByAnchoring
     * @see addSessionToCentralGraph
     *
     * @note The first element of _sess_pair is the index, and the second element is the session object.
     * @warning Ensure that the sessions_ map is populated with valid sessions before calling this function.
     */
    void addAllSessionsToGraph();

    /**
     * @brief Initializes the trajectory by anchoring a pose node based on the provided session.
     *
     * This function initializes the trajectory of a session by anchoring it to the anchor node.
     * The anchor node index is determined using the session index and a predefined offset.
     * For the central session, a prior factor is added with a very low model for the covariance,
     * while for other sessions (query session), a prior factor is added with a larger noise.
     * Both anchor nodes are also added to the initial estimate with the poseOrigin.
     *
     * @param _sess The Session object containing information about the session.
     *
     * @note The anchor node of the central session is added as a Prior Factor with a very low
     * model for the covariance, and the query with a large noise. Both take the PoseOrigin (0,0,0)
     * as prior.
     *
     * @param[in,out] gtSAMgraph The factor graph representing the trajectory.
     * @param[in,out] initialEstimate The initial estimate of the trajectory.
     */
    void initTrajectoryByAnchoring(const Session& _sess);

    /**
     * @brief Adds a session to the central graph.
     *
     * This function adds a session to the central graph by first adding nodes and then edges.
     * Nodes are added based on the session's nodes, and edges are added based on the session's edges.
     * The function distinguishes between the initial node and odometry nodes, and adds Prior Factors accordingly.
     * One the initial nodes of each section are added to the graph.
     * If `do_not_edit_central_session_` is set, additional constraints are added to all the poses of the central session.
     * The number of Prior Factors added to the graph is printed if debug messages are enabled.
     *
     * @param _sess The session to be added to the central graph.
     *
     * @see genGlobalNodeIdx
     * @see PriorFactor
     * @see BetweenFactor
     *
     * @warning Ensure that the global graph (gtSAMgraph) and initial estimate (initialEstimate) are appropriately initialized before calling this function.
     */
    void addSessionToCentralGraph(const Session& _sess);


    /**
     * @brief Optimizes the multi-session graph using the iSAM2 solver.
     * The most important input for this function is "isam" which is the porblem
     * This function utilizes the iSAM2 solver for incremental smoothing and mapping to perform an several update steps.
     * The update involves adding new factors (gtSAMgraph) and new Theta (initialEstimate) to the system.
     *
     * The ultimate goal of the optimization is to calculate the configuration of the robot trajectory that is maximally consistent with the observations.
     * In graph SLAM, observations are represented as links between poses. The algorithm aims to find poses that converge to the minimum energy, and the result
     * is an estimate of all variables in the incomplete linear delta computed during the last update.
     *
     * The marginals  of the result (while not retrieved here) provide information on the trustworthiness of the solution
     * based on the accumulated evidence of the observations.
     *
     * @param _toOpt A boolean flag indicating whether to perform optimization. If set to false, the function returns without performing any optimization.
     *
     * @note The function internally updates the iSAM2 solver multiple times and logs information about the optimization process.
     * @note Debugging information, such as ROS_INFO_STREAM and console output, is provided and can be activated based on the is_display_debug_msgs_ flag.
     */
    // PART 2. Optimize graph
    void optimizeMultiSessionGraph(bool _toOpt);


    // PART 3. Detect LC with SC and Optimize again
    /** MV
     * @brief Detects inter-session ScanContext loops.
     *
     * This function identifies loop closures between a target and a source session using ScanContext.
     * It populates two sets of loop indices: SCLoopIdxPairs_ for successful loop closures and RSLoopIdxPairs_
     * for cases where a loop closure is not found in the target session.
     *
     * @param [out] SCLoopIdxPairs list of paris of SC loop closures
     * @param [out] RSLoopIdxPairs list of paris of possible RS loop closures -> basically rejected SC Loops because the min_dist was larger than SC_DIST_THRES
     *
     * @note The function assumes the existence of a global sessions_ vector and relevant session indices (central_sess_idx and query_sess_idx).
     * @note ScanContext information is extracted from session managers (scManager) of the target and source sessions.
     * @note The actual loop closure detection is performed by calling the detectLoopClosureIDBetweenSession_OLD method on target_scManager.
     *
     * @see SessionManager::detectLoopClosureIDBetweenSession_OLD
     *
     * @param central_sess_idx The index of the central session.
     * @param query_sess_idx The index of the query session.
     */
    void detectInterSessionSCloops();
    //    void detectInterSessionRSloops(); ->  MV:initially empty
    void addSCloops();

    /** MV
     * @brief Iterates over all the scans in the query session, then finds the K (given parameter) closest scans
     * in the central session, and perform GICP with them.
     * If there is a (very) high fitness score -> the relative pose (after ICP) is added as constrain between the sessions
     * To have a fair calculation of the fitness score for all the scans only points inside the map are used
     * (points that are not in the floor of the reference map are removed)
     */
    void addKNNloops(); //  translating the target to source and doing YawGICP with submap from source session

    static std::vector<std::pair<int, int>> equisampleElements(const std::vector<std::pair<int, int>>& _input_pair, float _gap, int _num);

    /** MV
     * @brief The information gain is the reduction of entropy in the model (see slide 20 from here:
     * http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/18-igexplore.pdf)
     * or in wikipedia:
     * https://en.wikipedia.org/wiki/Information_gain_(decision_tree)
     *       
     */
    double calcInformationGainBtnTwoNodes(const int loop_idx_target_session, const int loop_idx_source_session);

    // PART 4. Detect LC with RS and Optimize again
    void findNearestRSLoopsTargetNodeIdx();
    bool addRSloops();


    /**
     * @brief Updates poses for all sessions based on ISAM2 estimates.
     *  It takes as inputs the following member variables
     * [in] sessions_ ( loaded sessions)
     * [in] isamCurrentEstimate (result after optimization)
     * This function iterates through all sessions, retrieves the anchor node transform from the ISAM2 estimate,
     * and then calls the `updateKeyPoses` function for each session to update the key poses in the central (global) coordinate system.
     *
     * @note The function assumes that the `genAnchorNodeIdx` function is defined to calculate the anchor node's global index.
     * @note Debugging information (cout) is commented out but can be uncommented for debugging purposes.
     */
    void updateSessionsPoses();

    std::optional<gtsam::Pose3> doICPVirtualRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);

    std::optional<gtsam::Pose3> doICPGlobalRelative(Session &target_sess, Session &source_sess,
            const int &loop_idx_target_session, const int &loop_idx_source_session);

    std::optional<gtsam::Pose3> doICPVirtualRelative_with_BIM_pc_v1( // For RS loop
            Session &target_sess, Session &source_sess,
            const int &loop_idx_target_session, const int &loop_idx_source_session, const size_t &index);

    std::optional<gtsam::Pose3> doICPVirtualRelative_with_BIM_pc_v2_Yaw_XY_porjection(Session& target_sess, Session& source_sess,
                                                                                      const int& loop_idx_target_session,
                                                                                      const int& loop_idx_source_session,
                                                                                      const float &yaw_angle_source,
                                                                                      const size_t & index);


    std::optional<gtsam::Pose3> doICPVirtualRelative_v3_and_v4(Session& target_sess, Session& source_sess,
                                                                                      const int& loop_idx_target_session,
                                                                                      const int& loop_idx_source_session,
                                                                                      const float &yaw_angle_source,
                                                                                                                const size_t & index, const int version);

    std::optional<gtsam::Pose3> doICPVirtualGlobal_v3_for_KNN(Session &target_sess, Session &source_sess, const int &loop_idx_target_session, const int &loop_idx_source_session, const size_t &index, const int version);

    // saver
    gtsam::Pose3 getPoseOfIsamUsingKey(Key _key); // MV never used in original code

    /**
     * @brief Writes the trajectories of all sessions to files in local and central coordinate systems.
     *
     * This function iterates through the estimated poses for each node in the graph,
     * distinguishing between anchor nodes and general nodes.
     *      For anchor nodes, it stores the poses as anchor transforms,
     *      and for general nodes, it collects the poses.
     * The collected poses are then written to two separate files for each session:
     * one in the local coordinate system and another in the central coordinate system,
     * after transforming them using the corresponding anchor transform.
     *
     * @param _postfix A postfix string to be appended to the output filenames (e.g.: "bfr_intersession_loops" or"aft_intersession_loops").
     *
     * @note The function relies on the iSAM2 solver and assumes that the graph optimization has been performed before calling this function.
     * @note File names are generated based on session names, and trajectories are saved in both local and central coordinate systems.
     */
    void writeAllSessionsTrajectories(const std::string& _postfix);
    void writeAllSessionsTrajectories_v2(const std::string _postfix);

    void writeAftSessionsTrajectoriesForCloudCompare(const std::string _postfix);
    void writeAftSessionsTrajectoriesForCloudCompareWithColor(const std::string _postfix, std::vector<int> _color);


    /**
     * @brief Parse and collect session anchor transforms and general node poses from the current estimate.
     *
     * This function iterates through the key-value pairs in the current estimate obtained from the iSAM2 solver.
     * It parses the session and anchor node indices, extracts the corresponding poses,
     * and organizes them into data structures for further processing.
     *
     * @param isamCurrentEstimate The current estimate of all nodes obtained from the iSAM2 solver.
     * @param[out] parsed_anchor_transforms A map storing anchor transforms for each session.
     * @param[out] parsed_poses A map storing lists of poses for general nodes in each session.
     *
     * @note The function assumes that the necessary data structures (parsed_anchor_transforms and parsed_poses) are appropriately initialized before the call.
     */
    void parseCurrentEstimate(const gtsam::Values& isamCurrentEstimate,
                              std::map<int, gtsam::Pose3>& parsed_anchor_transforms,
                              std::map<int, std::vector<gtsam::Pose3>>& parsed_poses);


    /**
     * @brief Write trajectories to files in local and central coordinate systems for each session.
     *
     * This function writes trajectories for each session to two separate files: one for local poses and another for central poses.
     * It retrieves the anchor transform for each session, and for each general node pose in the session,
     * it writes the local pose and the corresponding central pose to the respective files.
     *
     * @param parsed_poses A map storing lists of poses for general nodes in each session.
     * @param parsed_anchor_transforms A map storing anchor transforms for each session.
     * @param session_names A map mapping session indices to their corresponding names.
     * @param save_directory The directory where the trajectory files will be saved.
     * @param _postfix A postfix string to be appended to the output filenames (default is an empty string).
     *
     * @note The function assumes that the necessary data structures (parsed_poses, parsed_anchor_transforms, and session_names)
     *       are appropriately initialized before the call.
     */
    void writeTrajectories(const std::map<int, std::vector<gtsam::Pose3>>& parsed_poses,
                                    const std::map<int, gtsam::Pose3>& parsed_anchor_transforms,
                                    const std::map<int, std::string>& session_names,
                                    const std::string& save_directory,
                                    const std::string& _postfix);

    void addBetweenFactorWithAnchoring(int loop_idx_target_session,
                                               int loop_idx_source_session,
                                               const std::optional<gtsam::Pose3> & relative_pose_optional,
                                               const SharedNoiseModel& noiseModel_ = nullptr );


    void performFinalIcpWithOriginalMap_v3();
    void performFinalICP_v3();
    // Define console colors
    const std::string BLUE_BOLD = "\033[1;34m";
    const std::string DEFAULT = "\033[0m";
    void printParams();
    void part0Init();
    void part1LoadSessionsAndGraph();
    void part1LoadSessionsAndGraphTumFiles();
    void part2OptimizeGraph();
    void part3DetectLCWithSCAndOptimize();
    void part4DetectLCWithRSAndOptimize();
    void part5FinalICPWithRefMap();
    static std::tuple<Eigen::Matrix4d, double, double> make_visual_icp_v5_clean_open3D(const open3d::geometry::PointCloud& source_cloud, const open3d::geometry::PointCloud& target_cloud, const open3d::geometry::PointCloud& visual_target_cloud, const Matrix4d& initial_guess, int iterations, double thr_max_dist,
                                                    bool visual=true, bool print_after_each_iteration=false);

}; // Slam2ref