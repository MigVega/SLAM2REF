//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Slam2ref.h"
#include <iomanip>


Slam2ref::Slam2ref()
    : params(&ParamServer::getInstance()), //here the parameters from the YAML config file will be loaded.
    poseOrigin(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0)))
{

    std::cout << std::fixed << std::setprecision(2); // sets the decimal precision to 2 places.
} // ctor

Slam2ref::~Slam2ref() {}
// dtor



void printCurrentTime() {
    std::time_t rawtime;
    std::tm* timeinfo;

    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);

    std::cout << "Current Time: " << std::put_time(timeinfo, "%T") << std::endl;
}

void Slam2ref::run(void) {
    Timer general_timer("TOTAL Pipeline");
    printCurrentTime();
    printParams();


    part0Init();
    part1LoadSessionsAndGraph();
    if(params->write_input_merged_query_session_as_point_cloud_)
    {
        auto &source_sess = sessions_.at(query_sess_idx);
        source_sess.saveTransformedScansAndCreatePCD(blue_color_utils, "_ORIGINAL");
    }

    if(write_merged_central_point_cloud_)
    {
        Timer t("Extra PART: Writing merged CENTRAL point cloud ");
        auto &central_sess = sessions_.at(central_sess_idx);
        central_sess.saveTransformedScansAndCreatePCD(green_color_utils);
    }
    part2OptimizeGraph();
    part3DetectLCWithSCAndOptimize();
    part4DetectLCWithRSAndOptimize();

    // Write results after SC loops
    writeAllSessionsTrajectories("aft_SC_intersession_loops");
    writeAftSessionsTrajectoriesForCloudCompareWithColor("aft_SC_intersession_loops", red_color);

    if(using_MV_performing_KNN_loops)
    {
        Timer t("PART 4.2. KNN Loops");
        addKNNloops();
        optimizeMultiSessionGraph(true);
    }
    // Write results after KNN loops
    writeAllSessionsTrajectories("aft_KNN_intersession_loops");
    writeAftSessionsTrajectoriesForCloudCompareWithColor("aft_KNN_intersession_loops", red_color);


    // Part 6: saving final scans of the query and central session

    if(write_merged_query_point_cloud_)
    {
        Timer t("Extra PART: Writing merged QUERY point cloud ");
        auto &source_sess = sessions_.at(query_sess_idx);
        source_sess.saveTransformedScansAndCreatePCD(blue_color_utils);
    }

    if(using_MV_performing_final_ICP) {
        part5FinalICPWithRefMap();
    }

    printCurrentTime();
}

void Slam2ref::printParams(){
    if(print_config_param)
    {
        logInfo_print("\n%-30s %-30s\n"
                 "%-30s %s\n"
                 "%-30s %s\n"
                 "%-30s %s\n"
                 "%-30s %s\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %d\n"
                 "%-30s %f\n"
                 "%-30s %f\n"
                 "%-30s %lf\n",
                 "Variable", "Value",
                 "sessions_dir_", sessions_dir_.c_str(),
                 "central_sess_name_", central_sess_name_.c_str(),
                 "query_sess_name_", query_sess_name_.c_str(),
                 "save_directory_", save_directory_.c_str(),
                 "is_display_debug_msgs_", is_display_debug_msgs_,
                 "display_debug_msgs_RS_loops_", display_debug_msgs_RS_loops_,
                 "do_not_edit_central_session_", do_not_edit_central_session_,
                 "using_MV_modified_version_of_ICP", using_MV_modified_version_of_ICP,
                 "using_MV_performing_final_ICP", using_MV_performing_final_ICP,
                 "load_only_scans_and_kitti_files", load_only_scans_and_kitti_files,
                 "load_yaml_without_ros", load_yaml_without_ros_,
                 "numberOfCores", numberOfCores,
                 "historyKeyframeSearchNum", historyKeyframeSearchNum,
                 "kNumSCLoopsUpperBound", kNumSCLoopsUpperBound,
                 "kNumRSLoopsUpperBound", kNumRSLoopsUpperBound,
                 "kICPFilterSize", kICPFilterSize,
                 "loopFitnessScoreThreshold", loopFitnessScoreThreshold,
                 "SC_DIST_THRES_", SC_DIST_THRES_);
    }
}
void Slam2ref::part0Init() {
    Timer t("PART 0. Init");
    std::filesystem::remove_all(save_directory_);
    std::filesystem::create_directories(save_directory_);
    copyFile(configPath_,save_directory_ );
    initOptimizer();
    initNoiseConstants();

}

void Slam2ref::part1LoadSessionsAndGraph() {
    Timer t("PART 1. Load sessions");
    loadAllSessions();
    addAllSessionsToGraph();
}


void Slam2ref::part2OptimizeGraph() {
    // PART 2. Optimize graph with the loaded sessions (anchore nodes and edges) -> however,
    // without intersession constraints -> it means that not alignment between sessions will be done yet,
    // only the separated sessions will be optimized
    Timer t("PART 2. Optimize graph");

    optimizeMultiSessionGraph(true);
    writeAllSessionsTrajectories("bfr_intersession_loops");
    writeAftSessionsTrajectoriesForCloudCompareWithColor("bfr_intersession_loops", green_color);
}

void Slam2ref::part3DetectLCWithSCAndOptimize() {
    Timer t("PART 3. Detect LC with SC and Optimize again");
    detectInterSessionSCloops(); // detectInterSessionRSloops is internally done in SC detection
    addSCloops();
    optimizeMultiSessionGraph(true); // optimize the graph with existing edges + SC loop edges -> loop closures between sessions
}

void Slam2ref::part4DetectLCWithRSAndOptimize() {
    if(!MV_skipp_RS_loopDetection_)
    {
        Timer t("PART 4. Detect LC with RS and Optimize again");
        bool toOpt = addRSloops(); // using the optimized estimates (rough alignment using SC)
        optimizeMultiSessionGraph(toOpt); // optimize the graph with existing edges + SC loop edges + RS loop edges
    }
    else
    {
        std::cout <<"RS loop detection will be skipped" << std::endl;
    }

}

void Slam2ref::part5FinalICPWithRefMap() {
    Timer t("PART 5. Final ICP with Original Map");
    // PART 5. Do ICP of all the scans in the query session with the corresponding submap of the central session
    performFinalIcpWithOriginalMap_v3();
}

