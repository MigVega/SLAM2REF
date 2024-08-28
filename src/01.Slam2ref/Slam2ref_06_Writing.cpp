//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Slam2ref.h"

//-----------------------------------------------------------------------------
//------------------------------ Writing UTILS --------------------------------
//-----------------------------------------------------------------------------

void Slam2ref::writeAllSessionsTrajectories(const std::string& _postfix)
{
    //1. Parse poses
    std::map<int, gtsam::Pose3> parsed_anchor_transforms;
    std::map<int, std::vector<gtsam::Pose3>> parsed_poses;

    isamCurrentEstimate = isam->calculateEstimate();

    for (const auto &key_value : isamCurrentEstimate)
    {
        int curr_node_idx = int(key_value.key); // typedef std::uint64_t Key

        std::vector<int> parsed_digits;
        collect_digits(parsed_digits, curr_node_idx);
        int session_idx = parsed_digits.at(0);
        int anchor_node_idx = genAnchorNodeIdx(session_idx);

        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&key_value.value);
        if (!p)
            continue;
        gtsam::Pose3 curr_node_pose = gtsam::Pose3{p->value()};

        if (curr_node_idx == anchor_node_idx)
        {
            // anchor node
            parsed_anchor_transforms[session_idx] = curr_node_pose;
        }
        else
        {
            // general (rest of) nodes
            parsed_poses[session_idx].push_back(curr_node_pose);
        }
    }

    // 2. Mapping Session Indices to Names: Create a mapping between session indices and their corresponding names.
    std::map<int, std::string> session_names;
    for (auto &_sess_pair : sessions_)
    {
        auto &_sess = _sess_pair.second;
        session_names[_sess.index_] = _sess.name_;
    }

    // 3. Writing Trajectories:
    // For each session, open two files for writing: one for local poses and another for central poses.
    //        Retrieve the anchor transform for the session.
    //        For each general node poses in the session,
    //        write the local pose and the corresponding central pose to the respective files
    for (auto &_session_info : parsed_poses)
    {
        int session_idx = _session_info.first;

        std::string filename_local = save_directory_ + session_names[session_idx] + "_local_" + _postfix + ".txt";
        std::string filename_central = save_directory_ + session_names[session_idx] + "_central_" + _postfix + ".txt";
        cout << filename_central << endl;

        std::fstream stream_local(filename_local.c_str(), std::fstream::out);
        std::fstream stream_central(filename_central.c_str(), std::fstream::out);

        gtsam::Pose3 anchor_transform = parsed_anchor_transforms[session_idx];
        for (auto &_pose : _session_info.second)
        {
            writePose3ToStream(stream_local, _pose);

            gtsam::Pose3 pose_central = anchor_transform * _pose; // se3 compose (oplus)
            writePose3ToStream(stream_central, pose_central);
        }
    }

} // writeAllSessionsTrajectories


void Slam2ref::writeAllSessionsTrajectories_v2(const std::string _postfix)
{
    //1. Parse poses
    std::map<int, gtsam::Pose3> parsed_anchor_transforms;
    std::map<int, std::vector<gtsam::Pose3>> parsed_poses;

    isamCurrentEstimate = isam->calculateEstimate();
    parseCurrentEstimate(isamCurrentEstimate, parsed_anchor_transforms, parsed_poses);

    // 2. Mapping Session Indices to Names: Create a mapping between session indices and their corresponding names.
    std::map<int, std::string> session_names;
    for (auto &_sess_pair : sessions_)
    {
        auto &_sess = _sess_pair.second;
        session_names[_sess.index_] = _sess.name_;
    }

    // 3. Writing Trajectories:
    writeTrajectories(parsed_poses, parsed_anchor_transforms, session_names, save_directory_, _postfix);

} // writeAllSessionsTrajectories



void Slam2ref::parseCurrentEstimate(const gtsam::Values& isamCurrentEstimate,
                                    std::map<int, gtsam::Pose3>& parsed_anchor_transforms,
                                    std::map<int, std::vector<gtsam::Pose3>>& parsed_poses)
{
    for (const auto& key_value : isamCurrentEstimate)
    {
        int curr_node_idx = int(key_value.key); // typedef std::uint64_t Key

        std::vector<int> parsed_digits;
        collect_digits(parsed_digits, curr_node_idx);// if curr_node_idx = 100122 -> 2, 2, 1, 0, 0, 1
        int session_idx = parsed_digits.at(0); // this has to be either 1 or 2
        int anchor_node_idx = genAnchorNodeIdx(session_idx);

        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&key_value.value);
        if (!p)
            continue;
        gtsam::Pose3 curr_node_pose = gtsam::Pose3{p->value()};

        if (curr_node_idx == anchor_node_idx)
        {
            // anchor node
            parsed_anchor_transforms[session_idx] = curr_node_pose;
        }
        else
        {
            // general (rest of) nodes
            parsed_poses[session_idx].push_back(curr_node_pose);
        }
    }
}


void Slam2ref::writeTrajectories(const std::map<int, std::vector<gtsam::Pose3>>& parsed_poses,
                                 const std::map<int, gtsam::Pose3>& parsed_anchor_transforms,
                                 const std::map<int, std::string>& session_names,
                                 const std::string& save_directory,
                                 const std::string& _postfix)
{

    // Iterate through parsed poses for each session.
    for (const auto& _session_info : parsed_poses)
    {
        int session_idx = _session_info.first;

        // Generate filenames for local and central trajectory files.
        std::string filename_local = save_directory + session_names.at(session_idx) + "_local_" + _postfix + ".txt";
        std::string filename_central = save_directory + session_names.at(session_idx) + "_central_" + _postfix + ".txt";
        std::cout << filename_central << std::endl;

        // Open files for writing.
        std::fstream stream_local(filename_local.c_str(), std::fstream::out);
        std::fstream stream_central(filename_central.c_str(), std::fstream::out);

        // Retrieve the anchor transform for the session.
        gtsam::Pose3 anchor_transform = parsed_anchor_transforms.at(session_idx);

        // Write local and central poses to the respective files.
        for (const auto& _pose : _session_info.second)
        {
            writePose3ToStream(stream_local, _pose);

            // Transform pose to central coordinates using the anchor transform.
            gtsam::Pose3 pose_central = anchor_transform * _pose; // SE(3) composition (oplus).
            writePose3ToStream(stream_central, pose_central);
        }
    }
}


void Slam2ref::writeAftSessionsTrajectoriesForCloudCompare(const std::string _postfix = "")
{
    // parse
    std::map<int, gtsam::Pose3> parsed_anchor_transforms;
    std::map<int, std::vector<gtsam::Pose3>> parsed_poses;

    isamCurrentEstimate = isam->calculateEstimate();
    for (const auto &key_value : isamCurrentEstimate)
    {

        int curr_node_idx = int(key_value.key); // typedef std::uint64_t Key

        std::vector<int> parsed_digits;
        collect_digits(parsed_digits, curr_node_idx);
        int session_idx = parsed_digits.at(0);
        int anchor_node_idx = genAnchorNodeIdx(session_idx);

        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&key_value.value);
        if (!p)
            continue;
        gtsam::Pose3 curr_node_pose = gtsam::Pose3{p->value()};

        if (curr_node_idx == anchor_node_idx)
        {
            // anchor node
            parsed_anchor_transforms[session_idx] = curr_node_pose;
        }
        else
        {
            // general nodes
            parsed_poses[session_idx].push_back(curr_node_pose);
        }
    }

    std::map<int, std::string> session_names;
    for (auto &_sess_pair : sessions_)
    {
        auto &_sess = _sess_pair.second;
        session_names[_sess.index_] = _sess.name_;
    }

    // write
    for (auto &_session_info : parsed_poses)
    {
        int session_idx = _session_info.first;

        std::string filename_local = save_directory_ + session_names[session_idx] + "_local_" + _postfix + "_CC.txt";
        std::string filename_central = save_directory_ + session_names[session_idx] + "_central_" + _postfix + "_CC.txt";
        cout << filename_central << endl;

        std::fstream stream_local(filename_local.c_str(), std::fstream::out);
        std::fstream stream_central(filename_central.c_str(), std::fstream::out);

        gtsam::Pose3 anchor_transform = parsed_anchor_transforms[session_idx];
        // counter = 1;
        // std::vector red_color = [1 ,0 ,0 ]
        // std::vector blue_color = [0 ,0 ,1 ]

        for (auto &_pose : _session_info.second)
        {
            // if(counter == 4 || counter == 8 ||  counter ==12)
            // {
            writePose3ToStream_CC(stream_local, _pose);
            // writePose3ToStream_CC_withColor(stream_local, _pose, red_color);

            gtsam::Pose3 pose_central = anchor_transform * _pose; // se3 compose (oplus)
            writePose3ToStream_CC(stream_central, pose_central);
            // writePose3ToStream_CC_withColor(stream_central, pose_central, blue_color);
            // }
            // counter++;
        }
    }

}

void Slam2ref::writeAftSessionsTrajectoriesForCloudCompareWithColor(const std::string _postfix = "", std::vector<int> _color = {225, 225, 225})
{
    // parse
    std::map<int, gtsam::Pose3> parsed_anchor_transforms;
    std::map<int, std::vector<gtsam::Pose3>> parsed_poses;

    isamCurrentEstimate = isam->calculateEstimate();
    for (const auto &key_value : isamCurrentEstimate)
    {

        int curr_node_idx = int(key_value.key); // typedef std::uint64_t Key

        std::vector<int> parsed_digits;
        collect_digits(parsed_digits, curr_node_idx);
        int session_idx = parsed_digits.at(0);
        int anchor_node_idx = genAnchorNodeIdx(session_idx);

        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&key_value.value);
        if (!p)
            continue;
        gtsam::Pose3 curr_node_pose = gtsam::Pose3{p->value()};

        if (curr_node_idx == anchor_node_idx)
        {
            // anchor node
            parsed_anchor_transforms[session_idx] = curr_node_pose;
        }
        else
        {
            // general nodes
            parsed_poses[session_idx].push_back(curr_node_pose);
        }
    }

    std::map<int, std::string> session_names;
    for (auto &_sess_pair : sessions_)
    {
        auto &_sess = _sess_pair.second;
        session_names[_sess.index_] = _sess.name_;
    }

    std::vector<int> color;

    // write results
    for (auto &_session_info : parsed_poses)
    {
        int session_idx = _session_info.first;

        cout << "\nThe resulted poses of session Nr." << session_idx << " will be saved on: " << endl;
        std::string filename_local = session_names[session_idx] + "_local_" + _postfix + "_CC.txt";
        std::string filename_central = "!" + session_names[session_idx] + "_central_" + _postfix + "_CC.txt"; // MV: adding the "!"
        // to have them on the top of the file list
        cout << " " << filename_central << "\n" << endl;

        std::fstream stream_local(save_directory_ + filename_local.c_str(), std::fstream::out);
        std::fstream stream_central( save_directory_ + filename_central.c_str(), std::fstream::out);

        gtsam::Pose3 anchor_transform = parsed_anchor_transforms[session_idx];

        // Giving color to the results
        if (session_idx == 1)
        { // the Central session will have a darker color (red or green) than the query session
            int substraction = 100 * (session_idx);
            color = {std::max(0, _color[0] - substraction), std::max(0, _color[1] - substraction), std::max(0, _color[2] - substraction)};
        }
        else
        {
            color = _color;
        }

        for (auto &_pose : _session_info.second)
        { // for each pose in the seesion write its translation and rotation in a text file

            writePose3ToStream_CC_withColor(stream_local, _pose, color);
            gtsam::Pose3 pose_central = anchor_transform * _pose; // se3 compose (oplus)
            writePose3ToStream_CC_withColor(stream_central, pose_central, color);
        }
    }

}