// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Slam2ref.h"
//-----------------------------------------------------------------------------
//---------------------     LOOP CLOSURE UTILS --------------------------------
//-----------------------------------------------------------------------------


void Slam2ref::detectInterSessionSCloops() // using ScanContext
{
    // Part 1: Get references to the central and source sessions
    auto &central_sess = sessions_.at(central_sess_idx);
    auto &source_sess = sessions_.at(query_sess_idx);

    // Part 2: Clear existing loop index pairs
    SCLoopIdxPairs_.clear();
    RSLoopIdxPairs_.clear();

    // Part 3: Get references to ScanContext managers of the central and source sessions
    auto &central_scManager = central_sess.scManager;
    auto &source_scManager = source_sess.scManager;
    int NO_LOOP_FOUND = -1;

    // Part 4: Iterate through nodes in the source ScanContext
    // Detect loop closures: Find loop-edge index pairs
    for (int source_node_idx = 0; source_node_idx < int(source_scManager.polarcontexts_.size()); source_node_idx++)
    {
        // Part 4.1: Extract key and descriptor information from the source ScanContext
        std::vector<float> source_node_invariant_ring_key = source_scManager.polarcontext_invariantkeys_mat_.at(source_node_idx);
        Eigen::MatrixXd source_node_scd = source_scManager.polarcontexts_.at(source_node_idx);

        // MV: the following is the important line
        // Part 4.2: Perform loop closure detection using central ScanContext
        // inputs: first: invariant ring key as a vector, second: actual ScanContext descriptor
        // output: detectResult: first: nn index of the loop, second: yaw diff
        auto detectResult = central_scManager.detectLoopClosureIDBetweenSession_v2(source_node_invariant_ring_key, source_node_scd);

        int loop_idx_source_session = source_node_idx;
        int loop_idx_central_session = detectResult.first;
        float yaw_angle_radians = detectResult.second;

        // Part 4.3: Handle cases where a loop closure is not found in the central session
        // Checks if a loop closure was not found (-1).
        // If so, it adds the loop index of the source session to RSLoopIdxPairs_ and continues to the next iteration.
        if (loop_idx_central_session == NO_LOOP_FOUND)
        {
            RSLoopIdxPairs_.emplace_back(-1, loop_idx_source_session); // -1 will later be found (nn pose).
            if(display_debug_msgs_RS_loops_)
                logInfo_cout("MV: RSLoopIdxPairs_.size() = ", RSLoopIdxPairs_.size());
            continue;
        }

        // Part 4.4: Add successful loop closure index pairs to SCLoopIdxPairs_
        SCLoopIdxPairs_.emplace_back(loop_idx_central_session, loop_idx_source_session);
        SCLoop_yaw_angles_.emplace_back(yaw_angle_radians);
    }
    // Part 5: Output the total number of inter-session loops found
    logInfo_cout("\033[1;32m Total " , SCLoopIdxPairs_.size() , " (SCLoopIdxPairs_.size()) inter-session loops were found. \033[0m");
} // detectInterSessionSCloops


void writePairsToFile(const std::vector<std::pair<int, int>>& pairs, const std::string& filename) {
    // Open the file for writing
    std::ofstream outputFile(filename);

    // Check if the file is opened successfully
    if (!outputFile.is_open()) {
        std::cerr << "Error: Could not open file '" << filename << "' for writing." << std::endl;
        return;
    }

    // Write each pair to the file
    for (const auto& pair : pairs) {
        outputFile << pair.first << " " << pair.second << std::endl;
    }

    // Close the file
    outputFile.close();
}

void writeFloatsToFile(const std::vector<float>& floats, const std::string& filename) {
    // Open the file for writing
    std::ofstream outputFile(filename);

    // Check if the file is opened successfully
    if (!outputFile.is_open()) {
        std::cerr << "Error: Could not open file '" << filename << "' for writing." << std::endl;
        return;
    }

    // Write each float value to the file
    for (const auto& value : floats) {
        outputFile << value << std::endl;
    }

    // Close the file
    outputFile.close();
}

void Slam2ref::addSCloops()
{
    if (SCLoopIdxPairs_.empty())
        return;

    //Writing the pairs in a text file
    std::string filename = save_directory_ + "SCpairs.txt";
    writePairsToFile(SCLoopIdxPairs_, filename);
    std::cout << "Pairs written to " << filename << " successfully." << std::endl;

    // equisampling sc loops
    int num_scloops_all_found = int(SCLoopIdxPairs_.size());
    int num_scloops_to_be_added = std::min(num_scloops_all_found, kNumSCLoopsUpperBound);
    int equisampling_gap = num_scloops_all_found / num_scloops_to_be_added; // 217/200 = 1.085  -> 1

    auto sc_loop_idx_pairs_sampled = equisampleElements(SCLoopIdxPairs_, static_cast<float>(equisampling_gap), num_scloops_to_be_added); // MV in this case is simply taking the first 200 SC loop pairs (because num_scloops_to_be_added = 1)
    auto num_scloops_sampled = static_cast<size_t>(sc_loop_idx_pairs_sampled.size()); // MV: This should be the same as num_scloops_to_be_added

    //Writing the equisample pairs in a text file
    filename = save_directory_ + "SCpairs_2_equisample.txt";
    writePairsToFile(sc_loop_idx_pairs_sampled, filename);
    std::cout << " equisample Pairs written to " << filename << " successfully." << std::endl;


    filename = save_directory_ + "SCpairs_yaws.txt";
    writeFloatsToFile(SCLoop_yaw_angles_, filename);
    std::cout << " Yaw angles (in radians) written to " << filename << " successfully." << std::endl;

    // add selected sc loops
    auto &central_sess = sessions_.at(central_sess_idx);
    auto &source_sess = sessions_.at(query_sess_idx);

    std::vector<int> idx_added_loops;
    idx_added_loops.reserve(num_scloops_sampled);
    int counter = 0;
#pragma omp parallel for num_threads(numberOfCores) default(none) shared(sc_loop_idx_pairs_sampled, num_scloops_sampled, central_sess,source_sess, cout, idx_added_loops, counter )
    for (size_t ith = 0; ith < num_scloops_sampled; ith++)
    {
        auto &_loop_idx_pair = sc_loop_idx_pairs_sampled.at(ith);
        int loop_idx_central_session = _loop_idx_pair.first;
        int loop_idx_source_session = _loop_idx_pair.second;
        float yaw_angle_source_scan = SCLoop_yaw_angles_.at(ith);

        std::optional<gtsam::Pose3> relative_pose_optional;
        
        if(using_MV_modified_version_of_ICP){

            if (MV_ICP_type_in_SC_ == 1)
            {
                relative_pose_optional = doICPVirtualRelative_with_BIM_pc_v1(central_sess, source_sess,loop_idx_central_session, loop_idx_source_session,ith);
            }
            else if(MV_ICP_type_in_SC_ == 2)
            {
            relative_pose_optional = doICPVirtualRelative_with_BIM_pc_v2_Yaw_XY_porjection(central_sess, source_sess, loop_idx_central_session,loop_idx_source_session,yaw_angle_source_scan, ith);
            }
            else if(MV_ICP_type_in_SC_ == 3 ||MV_ICP_type_in_SC_ == 4 )
            {
                relative_pose_optional = doICPVirtualRelative_v3_and_v4(central_sess, source_sess, loop_idx_central_session,loop_idx_source_session,yaw_angle_source_scan, ith,MV_ICP_type_in_SC_);

            }
        } else{ // Using Original Lt_slam code
            relative_pose_optional = doICPVirtualRelative(central_sess, source_sess, loop_idx_central_session, loop_idx_source_session);                                                   
        }


        if (relative_pose_optional)
        {
            addBetweenFactorWithAnchoring(loop_idx_central_session, loop_idx_source_session,relative_pose_optional, robustNoise);
            counter++;
        }
    }

    // Part 5: Output the total number of inter-session loops found
    logInfo_cout("\033[1;32m Total ", counter , " (SCLoopIdxPairs_.size()) inter-session loops passed the ICP threshold and were added to the graph. \033[0m");
} // addSCloops

/**
 * @brief Equi-samples elements from the input pair vector based on the specified gap and number of samples.
 *
 * Given an input vector of pairs of integers and parameters for equisampling (gap and number of samples),
 * this function performs equisampling and returns a new vector containing equisampled pairs.
 *
 * @param _input_pair The input vector of pairs of integers to be equisampled.
 * @param _gap The gap between sampled elements.
 * @param _num_sampled The number of equisampled elements to generate.
 * @return A vector containing equisampled pairs from the input vector.
 */
std::vector<std::pair<int, int>> Slam2ref::equisampleElements(
        const std::vector<std::pair<int, int>> &_input_pair, float _gap, int _num_sampled)
{
    std::vector<std::pair<int, int>> sc_loop_idx_pairs_sampled;

//    int equisampling_counter{0};

    std::vector<int> equisampled_idx;
    for (int i = 0; i < _num_sampled; i++)
        equisampled_idx.emplace_back(std::round(float(i) * _gap));

    for (auto &_idx : equisampled_idx)
        sc_loop_idx_pairs_sampled.emplace_back(_input_pair.at(_idx));

    return sc_loop_idx_pairs_sampled;
}



// Function to add a BetweenFactorWithAnchoring to gtSAM graph and print a debug message
// LC type either SC or RS (is only use for the print)
void Slam2ref::addBetweenFactorWithAnchoring(int loop_idx_central_session,
                                             int loop_idx_source_session,
                                             const std::optional<gtsam::Pose3> & relative_pose_optional, const SharedNoiseModel& noiseModel_)
   {
    mtx.lock();

    int key1 = genGlobalNodeIdx(central_sess_idx, loop_idx_central_session);
    int key2 = genGlobalNodeIdx(query_sess_idx, loop_idx_source_session);
    int anchor_key1 = genAnchorNodeIdx(central_sess_idx);
    int anchor_key2 = genAnchorNodeIdx(query_sess_idx);

    const gtsam::Pose3& relative_pose = relative_pose_optional.value();
    gtSAMgraph.add(BetweenFactorWithAnchoring<gtsam::Pose3>(
            key1,
            key2,
            anchor_key1,
            anchor_key2,
            relative_pose,
            noiseModel_));

    mtx.unlock();

    //    // Debug message
    //    mtx.lock();
    ////    std::cout <<  "     " <<LC type << " loop detector found an inter-session edge between "
    //              << key1 << " and " << key2<< std::endl;
    ////              << " (anchor nodes are " << anchor_key1 << " and " << anchor_key2 << ")" << std::endl;
    //    mtx.unlock();
}