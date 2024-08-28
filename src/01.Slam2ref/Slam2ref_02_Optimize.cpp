//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#include "01.Slam2ref/Slam2ref.h"
/*// // // // // // // // // // // //
 * PART 2. Optimize graph
 */ // // // // // // // // // // // //

void Slam2ref::optimizeMultiSessionGraph(bool _toOpt)
{
    if (!_toOpt)
        return;
    /**
     * This code uses the iSAM2 (incremental smoothing and mapping) solver to perform an update step.
     * The update involves adding new factors (gtSAMgraph) and new Theta (initialEstimate) to the system
     * The iSAM2 algorithm is then run for a full step, re-linearizing and updating the solution as needed based on specified thresholds.
     * Optionally, existing factors can be removed from the system, and certain variables can be constrained, held constant, or re-eliminated.
     * The process is repeated multiple times, and information messages are logged to indicate the progress of the algorithm.
     */
     // The problem is like a mechanical problem with springs that are holding each other and then disturbed
     // -> and we want to find the poses that converge to the minimum energy (when the springs do not move anymore)
     // Ultimate goal: calculate the configuration of the robot trajectory that is maximally consistent with the robot trajectory
     // MV in graph SLAM observations are links between poses
     // MV: the following line is similar as initializing an optimizer  e.g.:(LevenbergMarquardOptimizer optimizer(graph, initialEstimate);

     // For more information about update here the source code: https://github.com/borglab/gtsam/blob/69a3a75195b65356d6e56669e5199d325c7962c9/gtsam/nonlinear/ISAM2.cpp#L414
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    // MV: Compute an estimate (on all variables) from the incomplete linear delta computed during the last update.
    // This delta is incomplete because it was not updated below wildfire_threshold.
    // If only a single variable is necessary, it is faster to call calculateEstimate(const KEY&).
    isamCurrentEstimate = isam->calculateEstimate(); // must be followed by update

    // MV: The marginals of the result will tell us how much we trust the solution, given the accumulated evidence of the observations

    // MV: I think the following resize and "clear" are needed to restart the variables gtSAMgraph containing the "factors"
    // and initialEstimate containing the "values" of the initial estimated poses (from odometry), since they were already added to the
    // isam variable (the problem)  in the first update, they are not needed anymore, and they have to be free to add the newest
    // -> intra-session loop closures.
    gtSAMgraph.resize(0); // MV: since this new size is less than the original -> all factors will be removed
    initialEstimate.clear();

    updateSessionsPoses();

    if (is_display_debug_msgs_)
    {
        // if this is activated, you will see in console something like in the following two lines:
        //      Current estimate: Values with 195 values:
        //      Value 1000000: (gtsam::Pose3) R:
        std::cout << "**********************************************" << std::endl;
        std::cout << "***** variable values after optimization *****" << std::endl;
        std::cout << std::endl;
        isamCurrentEstimate.print("Current estimate: ");
        std::cout << std::endl;
        // std::ofstream os("path/to/your/data/PoseGraphExample.dot");
        // gtSAMgraph.saveGraph(os, isamCurrentEstimate);
    }
} // optimizeMultiSessionGraph


void Slam2ref::updateSessionsPoses()
{
    for (auto &_sess_pair : sessions_)
    {
        auto &_sess = _sess_pair.second;

        if (_sess.is_central_session_ and do_not_edit_central_session_)
            continue; // In this case, the poses of the central session will not be modified

        gtsam::Pose3 anchor_transform = isamCurrentEstimate.at<gtsam::Pose3>(genAnchorNodeIdx(_sess.index_));
        // cout << anchor_transform << endl;
        _sess.updateKeyPoses(isam, anchor_transform);
    }
} // updateSessionsPoses
