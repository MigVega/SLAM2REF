// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl_conversions/pcl_conversions.h>

#include "01.Slam2ref/nanoflann.hpp"
#include "01.Slam2ref/KDTreeVectorOfVectorsAdaptor.h"

#include "01.Slam2ref/tictoc.h"
#include "01.Slam2ref/RosParamServer.h"
#include "0.Utils/ParamServer.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

//using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using SCPointType = pcl::PointXYZ; // MV: changed since intensity is not used here
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;



void CreateImageFromScDescriptor( Eigen::MatrixXd sc_descriptor );

// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
private:
    ParamServer* params; // to get prams from config file
public: 
    SCManager( ); //: ParamServer() { } // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.

    /** 
     * @brief MV: It creates a matrix that stores the point cloud descriptor which is calculated in a polar way (depending on the angle and beam) from the point cloud
     * every bin in the descriptor takes the highest z Value of the points that lie over that bin
     * @param [in] _scan_down the initial laser scan point cloud
     * @return matrix with polar descriptor
    */
    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );

    /**
     * @brief MV: In contrast with the original ScanContext, here the value of the bin is equal to 1 or 0 depending on
     * the number of points that get in the bin.
     * It is 1 if there are more than points_in_z_direction_threshold points ( 8 for 16 vertical rays (16/2)), however the points in the bin should also have a max z difference of 10 (i.e. they do not correspond to ceiling and floor)
     *  and after applying kmeans clustering of the points, with k = 2, the variance in the z direction is larger than 0.1 (10 cm)
     *  It is smaller it means that we have points from ceiling and floor.
     *  It is similar as analyzing the histogram of the z coordinate plot of the points in the bin there should not be only two
     *  groups of points with a small z variance in values.
     *
     * @param [in] _scan_down the initial laser scan point cloud
     * @param [in] z_thr is the minimum number of points in a bin to be able to consider the bin as part of the descriptor
     * (basically this will filter out horitzonal elements)
     * @return matrix with polar descriptor
    */
    Eigen::MatrixXd makeIndoorScancontext( pcl::PointCloud<SCPointType> & _scan_down, int z_thr );
    /** 
     * @brief rowwise mean vector. MV: it goes to every row in the point cloud descriptor Matrix and then calculates its mean value
     * @param [in] _desc is the desciptor calculated with the "makeScancontext" function
    */
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );

    /** 
     * @brief columwise mean vector. MV: similiar to makeRingkeyFromScancontext but for columns, i.e. it goes to every column in the point cloud descriptor Matrix and then calculates its mean value
     * @param [in] _desc is the desciptor calculated with the "makeScancontext" function
    */
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    /** 
     * @brief MV: This function is kind of the "main" here, it calls the others to create the SC_descriptors and Ring, and Sector, and saves them in 
     * private variables (which have an "_" at the end)
     * @param [in] _scan_down the initial laser scan point cloud
     * @return  polarcontexts_; 
                polarcontext_invariantkeys_;
                polarcontext_vkeys_;
                polarcontext_invariantkeys_mat_;
    */
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );

    // for Slam2ref
    // User-side API for multi-session
    void saveScancontextAndKeys( Eigen::MatrixXd _scd );

    void initializeBatchSearchTree(bool& is_tree_batch_made);
    void performKNNSearch(const std::vector<float>& curr_key, std::vector<size_t>& candidate_indexes) const;
    std::tuple<double, int, int> calculatePairwiseDistance(Eigen::MatrixXd& curr_desc, const std::vector<size_t>& candidate_indexes);
    std::pair<int, float> detectLoopClosureIDBetweenSession_v2(std::vector<float>& _curr_key, Eigen::MatrixXd& _curr_desc);

    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    const int NO_LOOP_FOUND = -1;
    // hyper parameters ()

    const int    PC_NUM_RING = params->PC_NUM_RING_; // 20 in the original paper (IROS 18) # MV: number of rings used to divide the max lidar lenght (PC_MAX_RADIUS), e.g. if this PC_MAX_RADIUS = 10 meter and number of Rings is 20, it means we have a ring every 0.5 m
    const int    PC_NUM_SECTOR = params->PC_NUM_SECTOR_; // 60 in the original paper (IROS 18)
    const double PC_MAX_RADIUS = params->PC_MAX_RADIUS_; //80.0; // 80 meter max in the original paper (IROS 18) // MV: changed to 10 m for indoor
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR); // num of columns 360/60 = 6. 360/20 = 18
//    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING); // MV: if PC_MAX_RADIUS = 20 and PC_MAX_RADIUS = 10 -> there are 0.5 m between rings -> it means that when ever there are scans with 0.5 m or more, their Polar SC will look very different

    // tree
     const int    NUM_CANDIDATES_FROM_TREE = params->NUM_CANDIDATES_FROM_TREE_; // 10 is enough. (refer the IROS 18 paper) // MV: changed _original 3

    // loop thresholds // MV TODO: pass the next variables (even all would be good) to the config YAML file
    const double SEARCH_RATIO = params->SEARCH_RATIO_;// 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check should be required for robustness)
    // const double SC_DIST_THRES = 0.3; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
   
    const double SC_DIST_THRES = params->SC_DIST_THRES_; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // MV: The following 3 variables are not used here -> they were designed to use SC for real-time loop closure in SLAM
        const int    TREE_MAKING_PERIOD_ = 10; // i.e., remaking tree frequency, to avoid non-mandatory every remaking,
        // to save time cost / in the LeGO-LOAM integration,
        // it is synchronized with the loop detection callback (which is 1Hz)
        // so it means the tree is updated every 10 sec.
        // But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
        int          tree_making_period_conter = 0;
        const int    NUM_EXCLUDE_RECENT = 30; // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok.
        const double LIDAR_HEIGHT = 2.0; // MV: this does not make any difference in IndoorSC.
        // lidar height: add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.


    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    // MV Rotational invariant -> since are the average of the rows -> assumes there are only yaw variations between scans -> not pith or roll
    std::vector<Eigen::MatrixXd> polarcontext_invariantkeys_;
    KeyMat polarcontext_invariantkeys_mat_;
    KeyMat polarcontext_invariantkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    bool is_tree_batch_made = false;
    std::unique_ptr<InvKeyTree> polarcontext_tree_batch_;

}; // SCManager


