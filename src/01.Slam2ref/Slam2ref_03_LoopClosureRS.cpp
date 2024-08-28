// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Slam2ref.h"

// MV:
// I do not need this (calcInformationGainBtnTwoNodes as done in RS loops)
// since I assume I already have a good alignment from the SC loops!
// -> therefore,
// it is unnecessary to search for the best alignment -> I only need to perform the registration
// and ensure that only registration with high Fitness Score passed
double Slam2ref::calcInformationGainBtnTwoNodes(const int loop_idx_target_session, const int loop_idx_source_session)
{
    auto pose_s1 = isamCurrentEstimate.at<gtsam::Pose3>(genGlobalNodeIdx(central_sess_idx, loop_idx_target_session)); // node: s1 is the central
    auto pose_s2 = isamCurrentEstimate.at<gtsam::Pose3>(genGlobalNodeIdx(query_sess_idx, loop_idx_source_session));
    auto pose_s1_anchor = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(central_sess_idx));
    auto pose_s2_anchor = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(query_sess_idx));

    gtsam::Pose3 hx1 = traits<gtsam::Pose3>::Compose(pose_s1_anchor, pose_s1); // for the updated jacobian, see line 60, 219, https://gtsam.org/doxygen/a00053_source.html
    gtsam::Pose3 hx2 = traits<gtsam::Pose3>::Compose(pose_s2_anchor, pose_s2);
    gtsam::Pose3 estimated_relative_pose = traits<gtsam::Pose3>::Between(hx1, hx2);

    gtsam::Matrix H_s1, H_s2, H_s1_anchor, H_s2_anchor;
    auto loop_factor = BetweenFactorWithAnchoring<gtsam::Pose3>(
            genGlobalNodeIdx(central_sess_idx, loop_idx_target_session), genGlobalNodeIdx(query_sess_idx, loop_idx_source_session),
            genAnchorNodeIdx(central_sess_idx), genAnchorNodeIdx(query_sess_idx),
            estimated_relative_pose, robustNoise);
    loop_factor.evaluateError(pose_s1, pose_s2, pose_s1_anchor, pose_s2_anchor,
                              H_s1, H_s2, H_s1_anchor, H_s2_anchor);

    gtsam::Matrix pose_s1_cov = isam->marginalCovariance(genGlobalNodeIdx(central_sess_idx, loop_idx_target_session)); // note: typedef Eigen::MatrixXd  gtsam::Matrix
    gtsam::Matrix pose_s2_cov = isam->marginalCovariance(genGlobalNodeIdx(query_sess_idx, loop_idx_source_session));

    // calc S and information gain
    gtsam::Matrix Sy = Eigen::MatrixXd::Identity(6, 6); // measurement noise, assume fixed
    gtsam::Matrix S = Sy + (H_s1 * pose_s1_cov * H_s1.transpose() + H_s2 * pose_s2_cov * H_s2.transpose());
    double Sdet = S.determinant();
    double information_gain = 0.5 * log(Sdet / Sy.determinant());

    return information_gain;
}

/**
 * @brief Find the nearest target nodes for loop closures based on information gain.
 *
 * This function iterates through the given loop closure pairs and searches for the nearest target nodes in the central session.
 * The selection is based on information gain calculated
 * between the source and target nodes.
 *
 * @note The function assumes the existence of certain variables and functions like
 *       RSLoopIdxPairs_, genGlobalNodeIdx(), genAnchorNodeIdx(), isamCurrentEstimate, poseDistance(),
 *       calcInformationGainBtnTwoNodes(), sessions_, central_sess_idx, query_sess_idx, and
 *       display_debug_msgs_RS_loops_.
 *
 * @see genGlobalNodeIdx, genAnchorNodeIdx, poseDistance, calcInformationGainBtnTwoNodes
 *
 * @return None. The valid loop closure pairs are stored in RSLoopIdxPairs_.
 */
void Slam2ref::findNearestRSLoopsTargetNodeIdx() // based-on information gain
{
    std::vector<std::pair<int, int>> validRSLoopIdxPairs;

    for (auto rsloop_idx_pair : RSLoopIdxPairs_)
    {
        // curr query pose
        auto rsloop_idx_source_session = rsloop_idx_pair.second;
        auto rsloop_global_idx_source_session = genGlobalNodeIdx(query_sess_idx, rsloop_idx_source_session);

        auto source_node_idx = rsloop_idx_source_session;
        auto query_pose = isamCurrentEstimate.at<gtsam::Pose3>(rsloop_global_idx_source_session);
        auto query_sess_anchor_transform = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(query_sess_idx));
        auto query_pose_central_coord = query_sess_anchor_transform * query_pose;

        // find nn pose idx in the target sess
        auto &target_sess = sessions_.at(central_sess_idx);
        std::vector<int> target_node_idxes_within_ball;
        for (int target_node_idx = 0; target_node_idx < int(target_sess.nodes_.size()); target_node_idx++)
        {
            auto target_pose = isamCurrentEstimate.at<gtsam::Pose3>(genGlobalNodeIdx(central_sess_idx, target_node_idx));
            if (poseDistance(query_pose_central_coord, target_pose) < 10.0) // 10 is a hard-coding for fast test
            {
                target_node_idxes_within_ball.push_back(target_node_idx);
                if (display_debug_msgs_RS_loops_)
                    cout << "(all) RS pair detected: " << target_node_idx << " <-> " << source_node_idx << endl;
            }
        }

        // if no nearest one, skip
        if (target_node_idxes_within_ball.empty())
            continue;

        // selected a single one having maximum information gain
        int selected_near_target_node_idx = 0;  // MV: I needed to add this = 0, because sometimes the information gain (below) was always 0 and the variable "selected_near_target_node_idx" was never rewritten, so the computer left it with the default value (that was a large number)
        double max_information_gain{0.0};
        for (int nn_target_node_idx : target_node_idxes_within_ball)
        {
            double this_information_gain = calcInformationGainBtnTwoNodes(nn_target_node_idx, source_node_idx);
            if (this_information_gain > max_information_gain)
            {
                selected_near_target_node_idx = nn_target_node_idx;
                max_information_gain = this_information_gain;
            }
        }
        if (display_debug_msgs_RS_loops_)
        {
            cout << "RS pair detected: " << selected_near_target_node_idx << " <-> " << source_node_idx << endl;
            cout << "info gain: " << max_information_gain << endl;
        }
        validRSLoopIdxPairs.emplace_back(selected_near_target_node_idx, source_node_idx);
    }

    // update
    RSLoopIdxPairs_.clear();
    RSLoopIdxPairs_.resize((int)(validRSLoopIdxPairs.size()));
    std::copy(validRSLoopIdxPairs.begin(), validRSLoopIdxPairs.end(), RSLoopIdxPairs_.begin());

}

bool Slam2ref::addRSloops()
{
    if (kNumRSLoopsUpperBound == 0)
        return false;

    cout << "RSLoopIdxPairs_.size() = " << RSLoopIdxPairs_.size() << endl;
    if (RSLoopIdxPairs_.empty()) {
        cout << "MV: there are not RSLoopIdxPairs, therefore addRSloops will be skipped" << endl;
        return false;
    }
    else
    {
        logInfo_cout("\033[1;32m Total " , RSLoopIdxPairs_.size()  ," (size of RSLoopIdxPairs) inter-session loops were found. \033[0m");
    }
    // find nearest target node idx
    findNearestRSLoopsTargetNodeIdx();

    logInfo_cout("\033[1;32m After findNearestRSLoopsTargetNodeIdx: Total " , RSLoopIdxPairs_.size()  , " (size of RSLoopIdxPairs) inter-session loops are still there. \033[0m");

    // parse RS loop src idx
    int num_rsloops_all_found = int(RSLoopIdxPairs_.size());
    if (num_rsloops_all_found == 0)
    {
        return false;
    }
    int num_rsloops_to_be_added = std::min(num_rsloops_all_found, kNumRSLoopsUpperBound);
    int equisampling_gap = num_rsloops_all_found / num_rsloops_to_be_added;

    auto rs_loop_idx_pairs_sampled = equisampleElements(RSLoopIdxPairs_, static_cast<float>(equisampling_gap), num_rsloops_to_be_added);
    auto num_rsloops_sampled = static_cast<int>(rs_loop_idx_pairs_sampled.size());

    cout << "num of RS pair: " << num_rsloops_all_found << endl;
    cout << "num of sampled RS pair: " << num_rsloops_sampled << endl;

    // add selected rs loops
    auto &target_sess = sessions_.at(central_sess_idx);
    auto &source_sess = sessions_.at(query_sess_idx);
    int counter = 0;
#pragma omp parallel for num_threads(numberOfCores) default(none) shared(cout,rs_loop_idx_pairs_sampled,num_rsloops_sampled,target_sess,source_sess, counter)
    for (int ith = 0; ith < num_rsloops_sampled; ith++)
    {
        auto &_loop_idx_pair = rs_loop_idx_pairs_sampled.at(ith);
        int loop_idx_target_session = _loop_idx_pair.first;
        int loop_idx_source_session = _loop_idx_pair.second;

        if (display_debug_msgs_RS_loops_) {
            std::cout << "\nMV: in (parallel) ICPGlobalRelative, ith = " << ith << std::endl;
        }
        auto relative_pose_optional = doICPGlobalRelative(target_sess, source_sess, loop_idx_target_session, loop_idx_source_session);

        if (relative_pose_optional)
        {
            addBetweenFactorWithAnchoring(loop_idx_target_session,loop_idx_source_session, relative_pose_optional, robustNoise);
            counter++;

        }
    }
    logInfo_cout("\033[1;32m Total " , counter , " (RSLoopIdxPairs_.size()) inter-session loops passed the ICP threshold and were added to the graph. \033[0m");
    return true;
} // addRSloops
