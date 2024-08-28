// Author:   Giseop Kim   paulgkim@kaist.ac.kr
//
// Enhanced by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//
#include "01.Slam2ref/Scancontext.h"
#include "01.Slam2ref/tictoc.h"


float deg2rad(float _degrees)
{
    return _degrees * M_PI / 180.0;
}


const float RAD_TO_DEG = 180.0 / M_PI;

float xy2theta(const float& _x, const float& _y) {
    float angle = RAD_TO_DEG * atan(_y / _x);

    if (_x < 0 && _y >= 0)
        return 180 - angle;

    if (_x < 0 && _y < 0)
        return 180 + angle;

    if (_x >= 0 && _y < 0)
        return 360 - angle;

    return angle;
}



MatrixXd circshift( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction 
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return 
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift


std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} // eig2stdvec

SCManager::SCManager()//: ParamServer()
        : params(&ParamServer::getInstance()) {
    // Optional: Other initialization code can go here.
}

double SCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);
        
        if( (col_sc1.norm() == 0) || (col_sc2.norm() == 0) )
            continue; // don't count this sector pair. 

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm()); //MV: cosine similarity

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC


int SCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey

/**
 * @brief Computes the distance and optimal shift between two ScanContext matrices.
 *
 * This function calculates the distance and optimal shift between two ScanContext matrices using a multi-step process:
 *
 * 1. Fast Align Using Variant Key:
 *    - Constructs variant keys from the ScanContext matrices using makeSectorkeyFromScancontext.
 *    - Performs a fast alignment using the variant keys to find the optimal shift.
 *
 * 2. Fast Columnwise Diff:
 *    - Defines a search space for shifts based on the computed optimal shift from step 1.
 *    - Iterates through the search space and calculates the direct ScanContext distance for each shift.
 *    - Finds the minimum distance and the corresponding optimal shift.
 *
 * The final result is returned as a std::pair<double, int>, where the first element is the minimum ScanContext distance,
 * and the second element is the optimal shift value.
 *
 * @param _sc1 The first ScanContext matrix.
 * @param _sc2 The second ScanContext matrix.
 * @return A std::pair<double, int> representing the minimum ScanContext distance and the optimal shift.
 *
 * @see makeSectorkeyFromScancontext
 * @see fastAlignUsingVkey
 * @see circshift
 * @see distDirectSC
 */
std::pair<double, int> SCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    // 1. Fast alignment using variant (sector/column-wise) key (not in original IROS18)
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. Fast column-wise difference
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext


MatrixXd SCManager::makeScancontext( pcl::PointCloud<SCPointType>& _scan_down )
{
    TicToc time_for_creating_descriptor;

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd polar_context_descriptor = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // within 2d plane
    int ring_idx, sector_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sector_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // taking maximum z. MV: It basically checks the value in the row ring_idx and
        // column sector_idx of the polar_context_descriptor,
        // and if it is lower than the z value of that point,
        // then that value is taken and saved in the polar_context_descriptor.
        if ( polar_context_descriptor(ring_idx-1, sector_idx-1) < pt.z ) // -1 means cpp starts from 0
            polar_context_descriptor(ring_idx-1, sector_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later). MV: if not point was found in a certain desciptor bin, then its value is set to zero (instead of leaving "NO_POINT = -1000").
    for ( int row_idx = 0; row_idx < polar_context_descriptor.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < polar_context_descriptor.cols(); col_idx++ )
            if( polar_context_descriptor(row_idx, col_idx) == NO_POINT )
                polar_context_descriptor(row_idx, col_idx) = 0;

    time_for_creating_descriptor.toc("PolarContext making");

    return polar_context_descriptor;
} // SCManager::makeScancontext


MatrixXd SCManager::makeIndoorScancontext( pcl::PointCloud<SCPointType>& scan_pc, int z_thr){
    int indoor_sc_version = params->INDOOR_SC_VERSION_;
    const int NO_POINT = -1000;
    double min_z_diff = 0.1;
    int num_pts_scan_pc = scan_pc.points.size();

    MatrixXd descriptor = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    MatrixXd descriptor_counter = MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);

    std::vector<std::vector<std::vector<double>>> descriptor_z_values(PC_NUM_RING, std::vector<std::vector<double>>(PC_NUM_SECTOR));
    SCPointType pt;
    float azim_angle, azim_range; // within 2d plane
    int ring_idx, sector_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_pc; pt_idx++)
    {
        pt.x = scan_pc.points[pt_idx].x;
        pt.y = scan_pc.points[pt_idx].y;
        pt.z = scan_pc.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::min(std::max(1, (int)(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), PC_NUM_RING) - 1;
        sector_idx = std::min(std::max(1, (int)(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), PC_NUM_SECTOR) - 1;

        descriptor_counter(ring_idx, sector_idx) += 1;

        if(descriptor_counter(ring_idx, sector_idx) > z_thr){
            descriptor_z_values[ring_idx][sector_idx].push_back(pt.z);
        }
    }

    for(int i = 0; i < PC_NUM_RING; i++){
        for(int j = 0; j < PC_NUM_SECTOR; j++){
            if(!descriptor_z_values[i][j].empty() &&
               (*std::max_element(descriptor_z_values[i][j].begin(), descriptor_z_values[i][j].end()) -
                *std::min_element(descriptor_z_values[i][j].begin(), descriptor_z_values[i][j].end())) > min_z_diff){
                if(indoor_sc_version > 2) // i.e. v3 (yes or not) or v4 (closer rings -> more certanty about position of elements -> larger values in the descriptor)
                {
                    descriptor(i, j) = 1;
                }
                else{ // indoor_sc_version == 2 or 1
                    descriptor(i, j) =  descriptor_counter(i, j);
                }

            }
        }
    }

    for(int i = 0; i < descriptor.rows(); i++){
        for(int j = 0; j < descriptor.cols(); j++){
            if(descriptor(i, j) == NO_POINT){
                descriptor(i, j) = 0;
            }
        }
    }
    if (indoor_sc_version == 4) { // v4 (closer rings -> more certainty about position of elements -> larger values in the descriptor)
        for (int i = 0; i < descriptor.rows(); ++i) {
            for (int j = 0; j < descriptor.cols(); ++j) {
                descriptor(i, j) = (descriptor.rows() - i)*descriptor(i, j);
            }
        }
    }
    if (indoor_sc_version == 2)  {
        //# normalizing each ring of the descriptor (1 -> max value and 0 for the minimum)
        //# this is to give the same priority to vertical elements (e.g. walls) that are far away from the sensor,
        //# than the ones that are closer (and therefore have more points in the same bin -> x,y coords)
        double min_val, max_val;

        for (int i = 0; i < descriptor.rows(); ++i) {
            min_val = descriptor.row(i).minCoeff();
            max_val = descriptor.row(i).maxCoeff();
            // avoid division by zero
            if (max_val > min_val) {
                descriptor.row(i) = (descriptor.row(i).array() - min_val) * 100 / (max_val - min_val);
            }
        }
    }

    return descriptor;
}

double calculateMaxZvariance(const std::vector<SCPointType>& points)
{
    int k = 2; // Number of clusters
    int attempts = 5; // Number of times the algorithm is executed using different initial labellings
    cv::Mat labels, centers;

    // Convert points to cv::Mat
    cv::Mat data(points.size(), 1, CV_32F);
    for(size_t i = 0; i < points.size(); i++)
        data.at<float>(i) = points[i].z;

    // Apply kmeans
    cv::kmeans(data, k, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0), attempts, cv::KMEANS_PP_CENTERS, centers);

    // Compute variance of z values separately in each cluster
    cv::Mat data_cluster1, data_cluster2;
    for (size_t i = 0; i < points.size(); i++)
    {
        if (labels.at<int>(i) == 0)
            data_cluster1.push_back(data.at<float>(i));
        else
            data_cluster2.push_back(data.at<float>(i));
    }

    double meanZ_cluster1 = cv::mean(data_cluster1)[0];
    double variance_cluster1 = cv::sum((data_cluster1 - meanZ_cluster1).mul(data_cluster1 - meanZ_cluster1))[0] / (data_cluster1.rows - 1);

    double meanZ_cluster2 = cv::mean(data_cluster2)[0];
    double variance_cluster2 = cv::sum((data_cluster2 - meanZ_cluster2).mul(data_cluster2 - meanZ_cluster2))[0] / (data_cluster2.rows - 1);

    // Return the maximum variance
    return std::max(variance_cluster1, variance_cluster2);
}



MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1); // MV: A vector could simpler represent this,
    // however, might be represented as a matrix for the later knn tree construction.
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean(); // it's calculating the mean value for each row of the _desc matrix and storing these mean values in the invariant_key matrix.
    }

    return invariant_key;
} // SCManager::makeRingkeyFromScancontext


MatrixXd SCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} // SCManager::makeSectorkeyFromScancontext


const Eigen::MatrixXd& SCManager::getConstRefRecentSCD(void)
{
    return polarcontexts_.back();
}


void SCManager::saveScancontextAndKeys( Eigen::MatrixXd _scd )
{
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( _scd );
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( _scd );
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey ); // converts the matrix to a vector

    polarcontexts_.push_back( _scd ); 
    polarcontext_invariantkeys_.push_back(ringkey );
    polarcontext_vkeys_.push_back( sectorkey );
    polarcontext_invariantkeys_mat_.push_back(polarcontext_invkey_vec );

} // SCManager::saveScancontextAndKeys

void CreateImageFromScDescriptor( Eigen::MatrixXd sc_descriptor )
{
  // Create an OpenCV image
  const int rows = sc_descriptor.rows();
  const int cols = sc_descriptor.cols();
  cv::Mat image(rows, cols, CV_8UC1); // type is the data type of the image 
  //(e.g. CV_8UC1 for a grayscale image, or 
  //CV_8UC3 for a color image with three channels).

  // Copy data from Eigen matrix to OpenCV image
  const double* data_ptr = sc_descriptor.data();
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      image.at<double>(i, j) = data_ptr[i * cols + j];
    }
  }

  // Show the image
  cv::imshow("Image", image);
  cv::waitKey(0);

}

void SCManager::makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down )
{
    Eigen::MatrixXd sc_descriptor;
    if(params->use_indoor_scan_context_)
    {
        sc_descriptor = makeIndoorScancontext( _scan_down, params->INDOOR_SC_Z_COUNTER_THRES_ ); // v1
    }
    else{
        sc_descriptor = makeScancontext( _scan_down );
    }

//    CreateImageFromScDescriptor( sc_descriptor );
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc_descriptor );
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc_descriptor );
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

    polarcontexts_.push_back( sc_descriptor ); 
    polarcontext_invariantkeys_.push_back(ringkey );
    polarcontext_vkeys_.push_back( sectorkey );
    polarcontext_invariantkeys_mat_.push_back(polarcontext_invkey_vec );

} // SCManager::makeAndSaveScancontextAndKeys




/**
 * @brief Initializes the batch search tree for polar context invariant ring keys.
 *
 * This function constructs the tree for searching polar context invariant ring keys in batch.
 * It is called only once during the first detection.
 *
 * @param is_tree_batch_made Flag indicating whether the batch search tree has been created.
 */
void SCManager::initializeBatchSearchTree(bool& is_tree_batch_made)
{
    polarcontext_invariantkeys_to_search_.clear();
    polarcontext_invariantkeys_to_search_.assign(polarcontext_invariantkeys_mat_.begin(), polarcontext_invariantkeys_mat_.end());

    polarcontext_tree_batch_.reset();
    polarcontext_tree_batch_ = std::make_unique<InvKeyTree>(PC_NUM_RING, polarcontext_invariantkeys_to_search_, 10);

    is_tree_batch_made = true;
}



/**
* @brief Performs K-Nearest Neighbors (KNN) search search for polar context invariant ring keys.
*
* This function makes use of an object's variables (like polarcontext_tree_batch_),
* so make sure those are setup properly before calling this function.
*
* @param curr_key (The current observation key) A reference to a vector of floats. This vector represents the
* "query" or the data for which we want to find the nearest neighbors.
*
* @param candidate_indexes A reference to an output parameter: A vector of size_t
* that will store the indices of the "query"'s nearest neighbors found in the search.
*
* @note The function will directly modify the parameter `candidate_indexes`, which is the indirect output of it.
*
* @return void
*/
void SCManager::performKNNSearch(const std::vector<float>& curr_key, std::vector<size_t>& candidate_indexes) const
{
    std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);
    nanoflann::KNNResultSet<float> KNN_search_result(NUM_CANDIDATES_FROM_TREE);

    // The .init method of the KNN_search_result object is called to set up the object for the search. It's given the addresses of the first elements of the candidate_indexes and out_dists_sqr vectors. It means that results of the search (indices of the nearest neighbors and their corresponding distances) will be stored in these two vectors.
    KNN_search_result.init(&candidate_indexes[0], &out_dists_sqr[0]);

    //This line performs the actual K-nearest neighbor search. It's done using the findNeighbors method of the object pointed to by the index member of the polarcontext_tree_batch_ object. The findNeighbors method takes three arguments

    polarcontext_tree_batch_->index->findNeighbors(KNN_search_result, &curr_key[0], nanoflann::SearchParams(10)); // flann::SearchParams object initialized with a leaf maximum check of 10, which is the number of leaves (in the kd-tree, which is likely used by nanoflann for indexing the data) to visit when searching for neighbors.
}

/**
 * @brief Calculates the pairwise distance and finds the optimal column-wise best-fit using cosine distance.
 *
 * This function computes the pairwise distance between the current descriptor and candidate polar contexts.
 * It finds the optimal column-wise best-fit using cosine distance.
 * This is similar as making (rough) point cloud registration but only changing the yaw angle
 *
 * @param curr_desc The current descriptor.
 * @param candidate_indexes vector with candidate indexes form the knn tree
 * @return A tuple containing the distance and alignment result.
 */
std::tuple<double, int, int> SCManager::calculatePairwiseDistance(Eigen::MatrixXd& curr_desc, const std::vector<size_t>& candidate_indexes)
{
    double min_dist = 10000000;
    int nn_align = 0;
    int nn_idx = 0;
    for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
    {
        MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate);

        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if (candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = static_cast<int>(candidate_indexes[candidate_iter_idx]);
        }
    }

    return std::make_tuple(min_dist, nn_align, nn_idx);
}


/**
 * @brief Detects loop closure ID between sessions using ScanContext.
 *
 * This is the main function that orchestrates the process of detecting loop closures between sessions using ScanContext.
 * It calls the individual functions for each step.
 *
 * @param _curr_key The current observation key.
 * @param _curr_desc The current descriptor.
 * @return A pair containing the ID of the node for which a loop was detected and yaw angle difference (in radiance) for which the best match was found.
 */

std::pair<int, float> SCManager::detectLoopClosureIDBetweenSession_v2(std::vector<float>& _curr_key, Eigen::MatrixXd& _curr_desc)
{
    int loop_id {NO_LOOP_FOUND};
    auto& curr_key = _curr_key;
    auto& curr_desc = _curr_desc;

    // Step 0: Initialize batch search tree for polar context invariant ring keys
    if (!is_tree_batch_made)
    {
        initializeBatchSearchTree(is_tree_batch_made);
    }

    // Step 1: K-nearest neighbors (KNN) search
    std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);
    performKNNSearch(curr_key, candidate_indexes);

    // Step 2: Calculate pairwise distance (in a for loop) and find optimal column-wise best-fit
    std::tuple<double, int, int> distance_result = calculatePairwiseDistance(curr_desc, candidate_indexes);

    // Retrieve the values from the tuple
    double min_dist = std::get<0>(distance_result);
    int nn_align = std::get<1>(distance_result);
    int nn_idx = std::get<2>(distance_result);


    // Step 3: Similarity threshold
    if (min_dist < SC_DIST_THRES) {
        printf("\033[32m    [GOOD] Condition min_dist < SC_DIST_THRES met!. Actual: %f < %f\033[0m\n", min_dist, SC_DIST_THRES);
        loop_id = nn_idx;
    } else {
        printf("    [BAD]  Condition min_dist < SC_DIST_THRES NOT met. Actual: %f < %f\n", min_dist, SC_DIST_THRES);
    }

    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result {loop_id, yaw_diff_rad};

    return result;
}




