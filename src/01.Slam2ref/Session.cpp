// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "01.Slam2ref/Session.h"

Session::Session() //: ParamServer()
{
//    params = &ParamServer::getInstance();
}

Session::Session(int _index, std::string _name, std::string _session_dir_path, bool _is_central_session, bool load_only_scans_and_kitti_files_)
    : index_(_index), name_(_name), session_dir_path_(_session_dir_path), is_central_session_(_is_central_session)
{
    params = &ParamServer::getInstance();
    allocateMemory();
    downSizeFilterICP.setLeafSize(params->kICPFilterSize, params->kICPFilterSize, params->kICPFilterSize);


    if (load_only_scans_and_kitti_files_)
    {
        loadSessionGraphKittiFiles(); // loads Kitti files

//        loadSessionKeyframePointclouds();   // loads PCD files -> saves them in cloudKeyFrames
        loadSessionKeyframePointCloudsInParallel();

        createSessionScanContextDescriptors(); // create SCD from the loaded PCD files -> saves them with scManager.makeAndSaveScancontextAndKeys(scd); -> which directly creates the ring keys (rows) and column-wise keys
    }
    else{
        loadSessionGraph(); // loads g2o files // only the same number of nodes that are in the g2o file will be loaded later as SCD

        loadSessionScanContextDescriptors(); // loads SCD files -> saves them with scManager.saveScancontextAndKeys(scd); -> which directly creates the ring keys (rows) and column-wise keys
        loadSessionKeyframePointclouds();   // loads PCD files -> saves them in cloudKeyFrames
    }

} // ctor


/* Memory allocation and Loading functions */

void Session::allocateMemory()
{
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    originPoses6D.reset(new pcl::PointCloud<PointTypePose>());
}

std::vector<std::string> split(const std::string& str, char delimiter){
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)){
        tokens.push_back(token);
    }
    return tokens;
}



void Session::loadSessionGraphKittiFiles()
{
    std::string posefile_path = session_dir_path_ + "/optimized_poses.txt";

    try
    {
        std::ifstream posefile_handle(posefile_path);

        // Check if the file is successfully opened
        if (!posefile_handle.is_open())
        {
            throw std::runtime_error("Unable to open file: " + posefile_path);
        }

        std::string strOneLine;


        Node prev_node, this_node;
        bool isFirstNode = true;
        int j = 0;

        while (getline(posefile_handle, strOneLine))
        {
            std::vector<std::string> line_info = split(strOneLine, ' ');

            if(line_info.size() >= 12) {
                gtsam::Matrix3 R;
                R << std::stod(line_info[0]), std::stod(line_info[1]), std::stod(line_info[2]),
                        std::stod(line_info[4]), std::stod(line_info[5]), std::stod(line_info[6]),
                        std::stod(line_info[8]), std::stod(line_info[9]), std::stod(line_info[10]);

                gtsam::Point3 T(std::stod(line_info[3]), std::stod(line_info[7]), std::stod(line_info[11]));

                this_node = { j, gtsam::Pose3(gtsam::Rot3(R), T ) };
                nodes_.insert(std::pair<int, Node>(j, this_node));

                if(!isFirstNode) {

                    gtsam::Pose3 poseFrom = prev_node.pose3;
                    gtsam::Pose3 poseTo = this_node.pose3;

                    gtsam::Pose3 relPose = poseFrom.between(poseTo); // same as in SC-A-LOAM

                    Edge this_edge = {j-1, j, relPose};

                    edges_.insert(std::pair<int, Edge>(j-1, this_edge)); // giseop for multimap
                }

                isFirstNode = false;
                prev_node = this_node;
                j = j + 1;
            }
        }

        // Initializes key poses in a 6D point format. The resulting key poses are stored in the
        //     * `cloudKeyPoses6D` and `originPoses6D` point clouds.
        initKeyPoses();

        //
//        logInfo_cout("\033[1;32m Session loaded: " << posefile_path << "\033[0m");
//        logInfo_cout("\033[1;32m - num nodes: " << nodes_.size() << "\033[0m");
        logInfo_cout("\033[1;32m Session loaded: ", posefile_path, "\033[0m");
        logInfo_cout("\033[1;32m - num nodes: ", nodes_.size(), "\033[0m");

    }
    catch(const std::runtime_error& re)
    {
        // Specific handling for runtime_error
        std::cerr << "Runtime error: " << re.what() << std::endl;
    }
    catch(const std::exception& ex)
    {
        // General catch for any other exceptions
        std::cerr << "Some exception has occurred: " << ex.what() << std::endl;
    }
} // loadSessionGraphKiitiFiles_withTrycatch


void Session::loadSessionGraphTumFiles()
{
    std::string posefile_path = session_dir_path_ + "/optimized_poses.tum";

    std::ifstream posefile_handle (posefile_path);
    std::string strOneLine;

    Node prev_node, this_node;
    bool isFirstNode = true;
    int j = 0;

    while (getline(posefile_handle, strOneLine))
    {
        std::vector<std::string> line_info = split(strOneLine, ' ');

        if(line_info.size() == 8) {
            this_node = { j,
                    gtsam::Pose3(
                            gtsam::Rot3(gtsam::Quaternion((double)std::stof(line_info[7]), (double)std::stof(line_info[4]),
                                                          (double)std::stof(line_info[5]), (double)std::stof(line_info[6]))), // xyzw to wxyz
                            gtsam::Point3((double)std::stof(line_info[1]), (double)std::stof(line_info[2]),
                                          (double)std::stof(line_info[3])))
            };

            nodes_.insert(std::pair<int, Node>((int)std::stof(line_info[0]), this_node));


            if(!isFirstNode) {
                // Calculate the difference in translation and rotation
                gtsam::Point3 trans = this_node.pose3.translation() - prev_node.pose3.translation();

                // Compute the conjugate of q2
                gtsam::Quaternion q2_conjugate = prev_node.pose3.rotation().toQuaternion().conjugate ();
                gtsam::Quaternion difference = this_node.pose3.rotation().toQuaternion() * q2_conjugate;
                Edge this_edge = {j-1, j, gtsam::Pose3( gtsam::Rot3(difference), trans)};

                edges_.insert(std::pair<int, Edge>(j-1, this_edge)); // giseop for multimap
            }

            isFirstNode = false;
            prev_node = this_node;
            j = j + 1;
        }
    }

    // Initializes key poses in a 6D point format. The resulting key poses are stored in the
    //     * `cloudKeyPoses6D` and `originPoses6D` point clouds.
    initKeyPoses();


    logInfo_cout("\033[1;32m Session loaded: ", posefile_path, "\033[0m");
    logInfo_cout("\033[1;32m - num nodes: ", nodes_.size(), "\033[0m");
//    logInfo_cout("\033[1;32m Session loaded: " << posefile_path << "\033[0m");
//    logInfo_cout("\033[1;32m - num nodes: " << nodes_.size() << "\033[0m");
} // loadSessionGraphTumFiles



void Session::loadSessionGraph()
{
    std::string posefile_path = session_dir_path_ + "/singlesession_posegraph.g2o";

    std::ifstream posefile_handle (posefile_path);
    std::string strOneLine;
    while (getline(posefile_handle, strOneLine))
    {
        G2oLineInfo line_info = splitG2oFileLine(strOneLine);

        // save variables (nodes)
        if( isTwoStringSame(line_info.type, G2oLineInfo::kVertexTypeName) ) {
            Node this_node { line_info.curr_idx,
                             gtsam::Pose3(
                                     gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0],
                                                                   line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                                     gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2]))
            };

            nodes_.insert(std::pair<int, Node>(line_info.curr_idx, this_node)); // giseop for multimap // MV TODO: remove redundancy the line_info.curr_idx is stored inside the node and in the pair of index, node
        }

        // save edges
        if( isTwoStringSame(line_info.type, G2oLineInfo::kEdgeTypeName) ) {
            Edge this_edge { line_info.prev_idx, line_info.curr_idx, gtsam::Pose3(
                    gtsam::Rot3(gtsam::Quaternion(line_info.quat[3], line_info.quat[0], line_info.quat[1], line_info.quat[2])), // xyzw to wxyz
                    gtsam::Point3(line_info.trans[0], line_info.trans[1], line_info.trans[2])) };
            edges_.insert(std::pair<int, Edge>(line_info.prev_idx, this_edge));  // MV: here the  prev_idx is simply the index of the first (starting) node of the edge
        }
    }
    // the next Initializes key poses in a 6D point format. The resulting key poses are stored in the
    //     * `cloudKeyPoses6D` and `originPoses6D` point clouds.
    initKeyPoses();

//    logInfo_cout("\033[1;32m Session loaded: " << posefile_path << "\033[0m");
//    logInfo_cout("\033[1;32m - num nodes: " << nodes_.size() << "\033[0m");
    logInfo_cout("\033[1;32m Session loaded: ", posefile_path, "\033[0m");
    logInfo_cout("\033[1;32m - num nodes: ", nodes_.size(), "\033[0m");
} // loadSessionGraph



void Session::initKeyPoses(void)
{
    for(auto & _node_info: nodes_)
    {
        PointTypePose thisPose6D;

        int node_idx = _node_info.first;    // first is simply the index of the node
        Node node = _node_info.second;      // the second contains the actual pose (however, this node also has an index)
        gtsam::Pose3 pose = node.pose3;

        thisPose6D.x = pose.translation().x();
        thisPose6D.y = pose.translation().y();
        thisPose6D.z = pose.translation().z();
        thisPose6D.intensity = node_idx;            // could be added
        thisPose6D.roll  = pose.rotation().roll();
        thisPose6D.pitch = pose.rotation().pitch();
        thisPose6D.yaw   = pose.rotation().yaw();
        thisPose6D.time = 0.0;

        cloudKeyPoses6D->push_back(thisPose6D); // this creates a point cloud with the key poses (nodes) it is used later to perform the ICP alignment
    }

    PointTypePose thisPose6D;
    thisPose6D.x = 0.0;
    thisPose6D.y = 0.0;
    thisPose6D.z = 0.0;
    thisPose6D.intensity = 0.0;
    thisPose6D.roll = 0.0;
    thisPose6D.pitch = 0.0;
    thisPose6D.yaw = 0.0;
    thisPose6D.time = 0.0;
    originPoses6D->push_back( thisPose6D );
} // initKeyPoses



void Session::createSessionScanContextDescriptors()
{

    // load SCDs
    int num_scd_loaded = 0;
    for (auto pcd: cloudKeyFrames)
    {
        scManager.makeAndSaveScancontextAndKeys(*pcd);
        num_scd_loaded++;
        if (num_scd_loaded >= static_cast<int>(nodes_.size())) { // only the nodes that are in the optimizied_poses will be loaded later as SCD
            break;
        }
    }
    cout << std::to_string(num_scd_loaded) << " SCDs are loaded for session: " << name_ << endl;

} // createSessionScanContextDescriptors


void Session::loadSessionScanContextDescriptors()
{
    std::string scd_dir = session_dir_path_ + "/SCDs/";

    // parse names (sorted)
    std::map<int, std::string> scd_names; // for auto-sort (because scManager should register SCDs in the right node order.)
    for(auto& _scd : fs::directory_iterator(scd_dir))
    {
        std::string scd_name = _scd.path().filename();

        std::stringstream scd_name_stream {scd_name};
        std::string scd_idx_str;
        getline(scd_name_stream, scd_idx_str, ',');
        int scd_idx = std::stoi(scd_idx_str);   // string to integer
        std::string scd_name_filepath = _scd.path();

        scd_names.insert(std::make_pair(scd_idx, scd_name_filepath));
    }

    // load SCDs
    int num_scd_loaded = 0;
    for (auto const& _scd_name: scd_names)
    {
        // std::cout << "load a SCD: " << _scd_name.second << endl;
        Eigen::MatrixXd scd = readSCD(_scd_name.second);
        scManager.saveScancontextAndKeys(scd);

        num_scd_loaded++;
        if (num_scd_loaded >= static_cast<int>(nodes_.size())) { // only the nodes that are in the g2o will be loaded later as SCD
            break;
        }
    }
    cout << "SCDs are loaded (" << name_ << ")" << endl;

} // loadSessionScanContextDescriptors



void Session::loadSessionKeyframePointclouds()
{
    std::string pcd_dir = session_dir_path_ + "/Scans/";

    // Check if directory exists
    if (!fs::exists(pcd_dir))
    {
        std::cerr << "Directory does not exist: " << pcd_dir << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::map<int, std::string> pcd_names; // for auto-sort (because scManager should register SCDs in the right node order.)

    try
    {
        for(auto& _pcd : fs::directory_iterator(pcd_dir))
        {
            std::string pcd_name = _pcd.path().filename();

            std::stringstream pcd_name_stream {pcd_name};
            std::string pcd_idx_str;
            getline(pcd_name_stream, pcd_idx_str, ',');
            int pcd_idx = std::stoi(pcd_idx_str);
            std::string pcd_name_filepath = _pcd.path();

            pcd_names.insert(std::make_pair(pcd_idx, pcd_name_filepath));

        }

        if (pcd_names.empty())
        {
            std::string error_message = "No .pcd files found in the directory: " + pcd_dir;
            throw std::runtime_error(error_message);
        }


        // load PCDs
        int num_pcd_loaded = 0;
        for (auto const& _pcd_name: pcd_names)
        {
            // cout << " load " << _pcd_name.second << endl;
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::io::loadPCDFile<PointType> (_pcd_name.second, *thisKeyFrame);
            cloudKeyFrames.push_back(thisKeyFrame);

            num_pcd_loaded++;
            if (num_pcd_loaded >= static_cast<int>(nodes_.size())) {
                break;
            }
        }
        cout << num_pcd_loaded << " Scans (Pcd files) were loaded for session: " << name_  << endl;
    }
    catch(const std::runtime_error& re)
    {
        // specific handling for runtime_error
        std::cerr << "Runtime error: " << re.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    catch(const std::exception& ex)
    {
        // general catch for any other exceptions
        std::cerr << "An exception has occurred: " << ex.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

const std::string SCANS_SUBDIRECTORY = "/Scans/";

void Session::populatePcdFilepaths(std::string& pcd_dir, std::map<int, std::string>& pcdFilepathsByIndex) {
    for(auto& pcd_file : fs::directory_iterator(pcd_dir)) {
        std::string pcd_filename = pcd_file.path().filename();
        std::stringstream pcd_filename_stream {pcd_filename};
        std::string pcd_index_str;
        getline(pcd_filename_stream, pcd_index_str, ',');

        int pcd_index = std::stoi(pcd_index_str);
        std::string pcd_filepath = pcd_file.path();

        pcdFilepathsByIndex.insert(std::make_pair(pcd_index, pcd_filepath));
    }
}

void Session::loadSessionKeyframePointCloudsInParallel() {
    std::string pcd_dir = session_dir_path_ + SCANS_SUBDIRECTORY;

    // Check if the directory exists
    if (!fs::exists(pcd_dir)) {
        std::cerr << "Directory does not exist: " << pcd_dir << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Parse filenames and collect them (sorted)
    std::map<int, std::string> pcdFilepathsByIndex;

    try {
        populatePcdFilepaths(pcd_dir, pcdFilepathsByIndex);

        if (pcdFilepathsByIndex.empty()) {
            std::string error_message = "No .pcd files found in the directory: " + pcd_dir;
            throw std::runtime_error(error_message);
        }

        // Load PCDs in parallel
        std::vector<std::vector<pcl::PointCloud<PointType>::Ptr>> tempKeyFrames(omp_get_max_threads());
        std::vector<std::pair<int, std::string>> entries(pcdFilepathsByIndex.begin(), pcdFilepathsByIndex.end());

#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(pcdFilepathsByIndex.size()); i++) {
            // Continue only if "i" is less than nodes_.size()
            if (i < static_cast<int>(nodes_.size())) {
                auto const& pcd_file = entries[i];
                pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
                pcl::io::loadPCDFile<PointType>(pcd_file.second, *thisKeyFrame);
                if(params->filter_scan_pc_with_radius_)
                {
                    // Apply the radius filter
                    double minRadius = 0.1; // could also be 0
                    double maxRadius = params->PC_MAX_RADIUS_;
                    thisKeyFrame = radiusFilterPointCloud(thisKeyFrame, minRadius, maxRadius);
                }
                if(params->filter_scan_pc_with_vertical_projection_)
                {
                    double gridSize = 0.1; // 10 cm
                    size_t minPointsInGrid = params->INDOOR_SC_Z_COUNTER_THRES_;
                    thisKeyFrame = filterPointsWithGrid(
                            thisKeyFrame,
                            gridSize,
                            minPointsInGrid);
                }

                int threadId = omp_get_thread_num();
                tempKeyFrames[threadId].push_back(thisKeyFrame);
            }
        }

        // Join all the vectors
        for (auto &tmp : tempKeyFrames) {
            cloudKeyFrames.insert(cloudKeyFrames.end(), tmp.begin(), tmp.end());
        }

        std::cout << entries.size() << " Scans (Pcd files) were loaded for session: " << name_  << std::endl;
    }
    catch(const std::runtime_error& re) {
        // specific handling for runtime_error
        std::cerr << "Runtime error: " << re.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    catch(const std::exception& ex) {
        // general catch for any other exceptions
        std::cerr << "An exception has occurred: " << ex.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
}




/* Utility functions -> that are not use in the session constructor (like the loaders) */

/**
 * @brief Updates key poses for the session based on ISAM2 estimates and anchor transform.
 *
 * This function iterates through the key poses of the session, updates their global indices,
 * transforms them from local to central coordinates using the provided anchor transform,
 * and updates the corresponding 6D pose information in the `cloudKeyPoses6D`.
 *
 * @param _isam Pointer to the ISAM2 object used for pose estimation.
 * @param _anchor_transform The transform to central coordinates for the anchor node.
 *
 * @note The function assumes that the `genGlobalNodeIdx` function is defined to calculate global node indices.
 * @note Debugging information (ROS_INFO_STREAM and cout) is commented out but can be uncommented for debugging purposes.
 */
void Session::updateKeyPoses(const gtsam::ISAM2 * _isam, const gtsam::Pose3& _anchor_transform)
{
    using gtsam::Pose3;

      gtsam::Values isamCurrentEstimate = _isam->calculateEstimate(); // Original_OLD

    // Iterates over all nodes in the session, takes the calculated estimated poses from the ISAM optimization (update)

    int numPoses = cloudKeyFrames.size();
    for (int node_idx_in_sess = 0; node_idx_in_sess < numPoses; ++node_idx_in_sess)
    {
        int node_idx_in_global = genGlobalNodeIdx(this->index_, node_idx_in_sess);
//        logInfo_cout("\033[1;35m MV: Part 3.2. updateKeyPoses: \n update the session " << index_ << "'s node: " << node_idx_in_sess << " (global idx: " << node_idx_in_global << ")\033[0m");
//        cout << "update the session " << index_ << "'s node: " << node_idx_in_sess << " (global idx: " << node_idx_in_global << ")" << endl;

         gtsam::Pose3 pose_self_coord = isamCurrentEstimate.at<Pose3>(node_idx_in_global);

        gtsam::Pose3 pose_central_coord = _anchor_transform.compose(pose_self_coord); // MV: changed because gtsam recommends it to better handle the multiplication

        cloudKeyPoses6D->points[node_idx_in_sess].x = static_cast<float>(pose_central_coord.translation().x());
        cloudKeyPoses6D->points[node_idx_in_sess].y = static_cast<float>(pose_central_coord.translation().y());
        cloudKeyPoses6D->points[node_idx_in_sess].z = static_cast<float>(pose_central_coord.translation().z());
        cloudKeyPoses6D->points[node_idx_in_sess].roll  = static_cast<float>(pose_central_coord.rotation().roll());
        cloudKeyPoses6D->points[node_idx_in_sess].pitch = static_cast<float>(pose_central_coord.rotation().pitch());
        cloudKeyPoses6D->points[node_idx_in_sess].yaw   = static_cast<float>(pose_central_coord.rotation().yaw());
    }
} // updateKeyPoses



gtsam::Pose3 Session::getPoseOfIsamUsingKey(const gtsam::ISAM2 * _isam, const gtsam::Key _key)
{
    const gtsam::Value &pose_value = _isam->calculateEstimate(_key);
    auto p_pose_value = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3> *>(&pose_value);
    gtsam::Pose3 pose = gtsam::Pose3{p_pose_value->value()};
    return pose;
}

void Session::loopFindNearKeyframesCentralCoord(
    pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
    const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(cloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes->clear(); // redundant?
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCentralCoord

/**
 * @brief Extracts and downsamples near keyframes in local coordinates around a specified keyframe.
 *
 * This function extracts a set of near keyframes in local coordinates around a given keyframe.
 * It considers a specified number of keyframes on either side of the given keyframe for extraction.
 * The extracted keyframes are then accumulated and downsampled using a downsampling filter.
 *
 * @param nearKeyframes [out] A pointer to the resulting point cloud containing the near keyframes.
 * @param key [in] The index of the target keyframe around which to extract near keyframes.
 * @param searchNum [in] The number of keyframes to consider on either side of the target keyframe.
 */
void Session::loopFindNearKeyframesLocalCoord(
    pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
    const int& key, const int& searchNum)
{
    // 1. Extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();

    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(cloudKeyFrames[keyNear], &originPoses6D->points[0]);
    }

    // 2. Check if nearKeyframes is empty after extraction
    if (nearKeyframes->empty())
        return;

    // 3. Downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes->clear(); // redundant? Clearing before copying from cloud_temp
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesLocalCoord

/**
 * @brief Extracts and downsamples near keyframes in local coordinates around a specified keyframe.
 *
 * This function extracts a set of near keyframes in local coordinates around a given keyframe.
 * It considers a specified number of keyframes on either side of the given keyframe for extraction.
 * The extracted keyframes are then accumulated and downsampled using a downsampling filter.
 *
 * @param nearKeyframes [out] A pointer to the resulting point cloud containing the near keyframes.
 * @param key [in] The index of the target keyframe around which to extract near keyframes.
 * @param searchNum [in] The number of keyframes to consider on either side of the target keyframe.
 */
void Session::loopFindNearKeyframesGeneral(
        pcl::PointCloud<PointType>::Ptr& nearKeyframes,
        const int& key, const int& searchNum, const std::string& local_or_central_coord_system)
{
    // 1. Extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();

    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        if (local_or_central_coord_system == "local")
            *nearKeyframes += *cloudKeyFrames[keyNear]; // MV: This should be the same as the previous is just tranforming with the identity matrix
        else
            *nearKeyframes += *transformPointCloud(cloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);
    }

    // 2. Check if nearKeyframes is empty after extraction
    if (nearKeyframes->empty())
        return;

    // 3. Downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    nearKeyframes->clear();
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesLocalCoord


void Session::createPointCloudWithIndices(
        pcl::PointCloud<PointType>::Ptr& outPointCloud,
        std::vector<int>& nearestIndices,  const std::string& local_or_central_coord_system)
{
    // 1. Extract near keyframes
    outPointCloud->clear();

    for (int nearestIndex : nearestIndices)
    {
        if (local_or_central_coord_system == "local")
            *outPointCloud += *cloudKeyFrames[nearestIndex];
        else
            *outPointCloud += *transformPointCloud(cloudKeyFrames[nearestIndex], &cloudKeyPoses6D->points[nearestIndex]);
    }

    // 3. Downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(outPointCloud);
    downSizeFilterICP.filter(*cloud_temp);
    *outPointCloud = *cloud_temp;
} // createPointCloudWithIndices



// In the following function the memory is optimized, saving each scan in a pcd file in the global coordinate system
// and loading them later for merging then the temp files are deleted
void Session::saveTransformedScansAndCreatePCD(const Eigen::Vector3i& color, const std::string& suffixName)
{

    std::string name = "00_merged_" + name_ + suffixName + ".pcd";
    std::string pcd_dir = params->save_directory_ + name;

    std::cout << "Creating merged map: " << name << std::endl;


    std::vector<std::string> temp_files;

    // Define a PointCloud with color
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedScanRGB(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Loop through all scans in the source session
    #pragma omp parallel for default(none) shared(nodes_, cloudKeyPoses6D, transformedScanRGB, temp_files)
    for(size_t i = 0; i < nodes_.size(); i++)
    {
        auto itr = nodes_.find(i);
        if (itr != nodes_.end()) {
            auto& kv = *itr;
            // Get the index of the node
            int k = kv.first;

            // Get initial pose for the node
            PointTypePose* pose_init = &cloudKeyPoses6D->points[k];

            // Get point cloud associated with the node and transform it using the initial pose
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedScan;
            transformedScan = transformPointCloud(cloudKeyFrames[k], pose_init);

            // Reset transformedScanRGB
            transformedScanRGB->clear();

            // Save this transformed and coloured scan to a temporary file
            std::string temp_file = params->save_directory_ + "_temp_transformed_" + std::to_string(k) + ".pcd";
            pcl::io::savePCDFile(temp_file, *transformedScan);
            #pragma omp critical
            {
                temp_files.push_back(temp_file);
            }
        } else {
            throw std::runtime_error("Key not found in multimap");
        }
    }
    // Parallelize the loading of temporary files
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> currentClouds(temp_files.size());

    std::cout << "      ...loading of temporary files to create final map" << std::endl;
#pragma omp parallel for default(none) shared(temp_files, currentClouds)
    for (size_t i = 0; i < temp_files.size(); i++)
    {
        currentClouds[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(temp_files[i], *currentClouds[i]) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read the temporary file \n");
        }
        // delete the temp file
        std::remove(temp_files[i].c_str());
    }

    for (const auto& currentCloud : currentClouds)
    {
        *finalCloud += *currentCloud;
    }

    // Create a VoxelGrid filter for downsampling
    pcl::VoxelGrid<PointType> voxel_grid_filter;

    // Set the input cloud to the filter
    voxel_grid_filter.setInputCloud(finalCloud);

    // Set the voxel grid leaf size
    float leaf_size = 0.1f;  // Leaf size of 10 cm
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    // Create a cloud to store the downsampled output
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>());

    // Apply the filter to get the downsampled output
    voxel_grid_filter.filter(*downsampled_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_downsampled_cloud = colorPointCloud(downsampled_cloud, color);

    // Save the final downsampled output as a PCD file
    pcl::io::savePCDFileBinary(pcd_dir, *colored_downsampled_cloud);

    std::cout << "Transformed scans merged and saved at: " << pcd_dir << std::endl;
}

void Session::moveCentralCloudToLocalCoordinates(pcl::PointCloud<PointType>::Ptr centralKeyframeCloud, const int &loop_idx_target_session)
{
    // Translate the source point cloud to the position of the central point cloud in global coordinate system
    // -> to be able to perform ICP with a well-constructed point cloud
    // -> not as done in VirtualICP after SC loop detection
    float translation_x = this->cloudKeyPoses6D->points[loop_idx_target_session].x;
    float translation_y = this->cloudKeyPoses6D->points[loop_idx_target_session].y;
    float translation_z = this->cloudKeyPoses6D->points[loop_idx_target_session].z;

    float r_yaw = this->cloudKeyPoses6D->points[loop_idx_target_session].yaw;
    float r_pitch = this->cloudKeyPoses6D->points[loop_idx_target_session].pitch;
    float r_roll = this->cloudKeyPoses6D->points[loop_idx_target_session].roll;

    // the central scan is moved (with 5 scans from the simulated scans)
    // from the global to the local coord system -> where the central scan is at 0,0,0
    PcTransformations::transformPointCloudWithAngles(std::move(centralKeyframeCloud), -translation_x, -translation_y, -translation_z, -r_yaw, -r_pitch, -r_roll);
}