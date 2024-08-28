// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#pragma once

#include "0.Utils/utility.h"
#include "01.Slam2ref/Scancontext.h"
#include "0.Utils/PcTransformations.h"

using namespace Slam3refParam;


struct Edge {
    int from_idx;
    int to_idx;
    gtsam::Pose3 relative;
};

struct Node {
    int idx;
    gtsam::Pose3 pose3;
};

using SessionNodes = std::multimap<int, Node>; // from_idx, Edge
using SessionEdges = std::multimap<int, Edge>; // from_idx, Edge


class Session // MV added inheritance from RosParamServer to get prams from config file
{
private:
    ParamServer* params;
public:

    int index_;

    std::string name_;
    std::string session_dir_path_;

    bool is_central_session_;

    SessionNodes nodes_;
    SessionEdges edges_;

    int anchor_node_idx_;

    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D; // store the poses of the nodes of key frames as point cloud, used for parsing submap represented in central coord system
    pcl::PointCloud<PointTypePose>::Ptr originPoses6D; // this is only the 0,0,0 pose

    std::vector<pcl::PointCloud<PointType>::Ptr> cloudKeyFrames; // stores the loaded PCD files of the key frames
    pcl::VoxelGrid<PointType> downSizeFilterICP;

    SCManager scManager;

public:
    Session();
    Session(int _index, std::string _name, std::string _session_dir_path, bool _is_central_session, bool load_only_scans_and_kitti_files_);

    /**
     * @brief Loads a pose graph from a G2O file, extracts node and edge information, and initializes the session's data structures.
     *
     * This function reads a G2O pose graph file, parses information about nodes and edges, and initializes the internal data structures
     * representing nodes and edges in the session. It also performs additional initialization related to key poses.
     *   The  key poses are stored in the  `cloudKeyPoses6D` and `originPoses6D` point clouds.
     *
     * @note The G2O file should be formatted with vertices and edges, and the file path is constructed based on the session directory path.
     * @note MV: reading the edges only gives additional information if there are loop closures in the edges,
     * otherwise the edges could be inferred from the ordered nodes.
     *
     * @see G2oLineInfo, Node, Edge, initKeyPoses.
     *
     * @return void
     *
     * @post The session's nodes and edges are populated based on the information from the G2O file.
     *
     * @code
     * // Example Usage:
     * Session session;
     * session.loadSessionGraph();
     * @endcode
     */
    void loadSessionGraph();
    void loadSessionGraphTumFiles();
    void loadSessionGraphKittiFiles();
    void loadSessionScanContextDescriptors();
    void createSessionScanContextDescriptors();
    void loadSessionKeyframePointclouds();
    void loadSessionKeyframePointCloudsInParallel();
    void populatePcdFilepaths(std::string& pcd_dir, std::map<int, std::string>& pcdFilepathsByIndex);

    /**
     * @brief Initializes key poses based on the nodes in the session.
     *
     * This function iterates through the nodes in the session, extracts relevant pose information,
     * and initializes key poses in a 6D point format. The resulting key poses are stored in the
     * `cloudKeyPoses6D` and `originPoses6D` point clouds.
     *
     * @note The `cloudKeyPoses6D` point cloud represents key poses associated with nodes, and the
     * `originPoses6D` point cloud contains a single pose representing the origin.
     *
     * @see Node, PointTypePose, cloudKeyPoses6D, originPoses6D
     *
     * @return void
     *
     * @post The `cloudKeyPoses6D` and `originPoses6D` point clouds are populated with key pose information.
     *
     * @code
     * // Example Usage:
     * Session session;
     * session.initKeyPoses();
     * @endcode
     */
    void initKeyPoses(void);

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
    void updateKeyPoses(const gtsam::ISAM2 * isam, const gtsam::Pose3& _anchor_transform);

    void loopFindNearKeyframesCentralCoord(
        pcl::PointCloud<PointType>::Ptr& nearKeyframes, 
        const int& key, const int& searchNum);
    void loopFindNearKeyframesLocalCoord(
        pcl::PointCloud<PointType>::Ptr& nearKeyframes,
        const int& key, const int& searchNum);

    // MV: the next function is to replace the previous two
    void loopFindNearKeyframesGeneral(
            pcl::PointCloud<PointType>::Ptr& nearKeyframes,
            const int& key, const int& searchNum, const std::string& local_or_central_coord_system = "central");

    void allocateMemory();

    gtsam::Pose3 getPoseOfIsamUsingKey(const gtsam::ISAM2 * _isam, gtsam::Key _key); // MV moved here from Slam2ref, still not used

    void createPointCloudWithIndices(
            pcl::PointCloud<PointType>::Ptr& outPointCloud,
            std::vector<int>& nearestIndices,  const std::string& local_or_central_coord_system); // MV added

    /**
     * @brief Saves transformed scans as a point cloud and creates a PCD file.
     *
     * This function takes a color vector as input and saves the transformed scans as a point cloud.
     * It loops through all scans in the source session, transforms each scan using the initial pose,
     * assigns color to each point in the transformed scan, and accumulates the transformed scans.
     * It then applies a VoxelGrid filter to downsample the point cloud, and saves the final downsampled output as a PCD file.
     *
     * @param color The color vector to assign to each point in the transformed scans.
     *
     * @see Session, transformPointCloud, pcl::VoxelGrid, pcl::io::savePCDFile
     *
     * @return void
     *
     * @post The transformed scans are merged and saved as a PCD file.
     *
     * @code
     * // Example Usage:
     * Eigen::Vector3i color;
     * saveTransformedScansAndCreatePCD(color);
     * @endcode
     */
    void saveTransformedScansAndCreatePCD(const Eigen::Vector3i& color, const std::string& suffixName = "");

    // Function that adjusts the cloud to local coordinates
    void moveCentralCloudToLocalCoordinates(pcl::PointCloud<PointType>::Ptr centralKeyframeCloud, const int &loop_idx_target_session);


}; // Session


// using Sessions = std::vector<Session>;
using Sessions = std::map<int, Session>;
using SessionsDict = std::map<std::string, Session>; // session name, session information

