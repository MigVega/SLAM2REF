//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
/*
 * PART 1. Load sessions and graph
 */
#include "01.Slam2ref/Slam2ref.h"

void Slam2ref::loadAllSessions()
{
    // MV: the following loop loads the query and central sessions which should be both in the sessions_dir (see params.yaml)
    logInfo_cout("\033[1;32m Load sessions' pose data from: " , sessions_dir_ ,"\033[0m");
    for (auto &_session_entry : fs::directory_iterator(sessions_dir_))
    {
        std::string session_name = _session_entry.path().filename();
        // MV: the next checks that the session name is either the central or the query session, it is not the case then it will continue
        //  It skips processing for entries whose filenames are neither equal to central_sess_name_ nor query_sess_name_.
        if (!isTwoStringSame(session_name, central_sess_name_) & !isTwoStringSame(session_name, query_sess_name_))
        {
            continue; // currently designed for only two-session. (could be generalized for N-session co-optimization)
        }

        // save a session (read graph txt file and load nodes and edges internally)
        int session_idx;
        if (isTwoStringSame(session_name, central_sess_name_))
            session_idx = central_sess_idx;  // in the header: const int central_sess_idx = 1;
        else
            session_idx = query_sess_idx;    // const int query_sess_idx = 2;

        std::string session_dir_path = _session_entry.path();

        // MV: the following loads all the nodes, edges (from the TUM files) and the PCD files of both sessions
        sessions_.insert(std::make_pair(session_idx,
                                        Session(session_idx, session_name, session_dir_path,
                                                isTwoStringSame(session_name, central_sess_name_), load_only_scans_and_kitti_files)));
    }

    std::cout << std::boolalpha; // When std::boolalpha is set, boolean values are output as true or false instead of "1" or ""0".
    logInfo_cout("\033[1;32m Total : ", sessions_.size() , " sessions are loaded.\033[0m");

    std::for_each(sessions_.begin(), sessions_.end(), [](auto &_sess_pair)
    { cout << " — " << _sess_pair.second.name_ << " (is central: " << _sess_pair.second.is_central_session_ << ")" << endl; });

} // loadSession


// the next is executed directly after the previous function -> it could be actually inside
void Slam2ref::addAllSessionsToGraph()
{
    for (auto &_sess_pair : sessions_)
    {
        auto &_sess = _sess_pair.second; // the second is actually the session,
        // the first is only the index
        initTrajectoryByAnchoring(_sess);
//        logInfo_cout("\033[1;35m Finished: initTrajectoryByAnchoring \033[0m");
        addSessionToCentralGraph(_sess);
//        logInfo_cout("\033[1;35m Finished: addSessionToCentralGraph \033[0m");
    }
} // addAllSessionsToGraph


void Slam2ref::initTrajectoryByAnchoring(const Session &_sess)
{
    // MV: in the next line the anchor node index will be retrieved ->
    // which is the session index (1 or 2) * the max number of nodes -> kSessionStartIdxOffset (now 1 million)
    int this_session_anchor_node_idx = genAnchorNodeIdx(_sess.index_);

    //  MV: the anchor node of the central session is added as a Prior Factor with a very low model for the covariance
    // and the query with a large noise

    // Both take as prior to the PoseOrigin which is 0,0,0
    if (_sess.is_central_session_)
    {
        gtSAMgraph.add(PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, priorNoise)); // PriorNoiseValue = 1e-102; (the smallest one)
    }
    else
    {
        gtSAMgraph.add(PriorFactor<gtsam::Pose3>(this_session_anchor_node_idx, poseOrigin, largeNoise));
    }

    // both anchor nodes are added with poseOrigin in the initial estimate (int index of anchor node, gtsam::pose3 Origin)
    initialEstimate.insert(this_session_anchor_node_idx, poseOrigin);
} // initTrajectoryByAnchoring



void Slam2ref::addSessionToCentralGraph(const Session &_sess)
{
    // First nodes will be added and then edges
    // 1. Add nodes
    int counter = 0;
    for (auto &_node : _sess.nodes_)
    {
        int node_idx = _node.second.idx; // the nodes where loaded in the constructor of the session (above)
        auto &curr_pose = _node.second.pose3;

        int curr_node_global_idx = genGlobalNodeIdx(_sess.index_, node_idx);

        // only the initial odom node will be added in the position right after the anchore node
        if (node_idx == 0)
        {
            // prior node
            // MV: only the first pose is added (again) as Prior factor with small Noise model (prior noise) but now with the curr_pose that was the result of the odometry module (the first node is usually at 0,0,0).
            gtSAMgraph.add(PriorFactor<gtsam::Pose3>(curr_node_global_idx, curr_pose, priorNoise));
            counter++;
        }
        else
        {
            if(do_not_edit_central_session_) // One of the main differences with LT_SLAM vs Slam2ref
            {
                // MV: adding constraints to all the poses of the central session so that it will not be edited by the optimization
                if (_sess.is_central_session_)
                {
                    gtSAMgraph.add(PriorFactor<gtsam::Pose3>(curr_node_global_idx, curr_pose, priorNoise));
                    counter++;
                }
            }
        }
        // odom nodes
        initialEstimate.insert(curr_node_global_idx, curr_pose);
    }
    if (is_display_debug_msgs_)
        cout << "\n\n...We added " << counter << " PriorFactors to the graph for the nodes in the session Nr." << _sess.index_ << " in addSessionToCentralGraph \n\n"
             << endl;

    // 2. Add Edges
    for (auto &_edge : _sess.edges_)
    {
        int from_node_idx = _edge.second.from_idx;
        int to_node_idx = _edge.second.to_idx;

        int from_node_global_idx = genGlobalNodeIdx(_sess.index_, from_node_idx);
        int to_node_global_idx = genGlobalNodeIdx(_sess.index_, to_node_idx);

        gtsam::Pose3 relative_pose = _edge.second.relative;
        if (std::abs(to_node_idx - from_node_idx) == 1) // MV: in case of consecutive nodes -> odometry -> odomNoise
        {
            // odom edge (temporally consecutive)
            gtSAMgraph.add(BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, odomNoise));
            if (is_display_debug_msgs_)
                cout << "add an odom edge between " << from_node_global_idx << " and " << to_node_global_idx << endl;
        }
        else // MV: in case of NOT consecutive nodes -> loop closure -> robustNoise
        {
            // loop edge
            gtSAMgraph.add(BetweenFactor<gtsam::Pose3>(from_node_global_idx, to_node_global_idx, relative_pose, robustNoise));
            if (is_display_debug_msgs_)
                cout << "add a loop edge between " << from_node_global_idx << " and " << to_node_global_idx << endl;
        }
    }
}

gtsam::Pose3 Slam2ref::getPoseOfIsamUsingKey(const Key _key)
{
    const Value &pose_value = isam->calculateEstimate(_key);
    auto p_pose_value = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&pose_value);
    gtsam::Pose3 pose = gtsam::Pose3{p_pose_value->value()};
    return pose;
}
