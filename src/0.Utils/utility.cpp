//
// Created by Miguel A. Vega-Torres 2024 miguel.vegatorres@gmail.com
//

#include "0.Utils/utility.h"

void readBin(const std::string& _bin_path, const pcl::PointCloud<PointType>::Ptr& _pcd_ptr)
{
 	std::fstream input(_bin_path.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << _bin_path << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
  
	for (int ii=0; input.good() && !input.eof(); ii++) {
		PointType point;

		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));

		_pcd_ptr->push_back(point);
	}
	input.close();
}


namespace Slam3refParam {
    // NOTE: kSessionStartIdxOffset in utility.h

    int ungenGlobalNodeIdx (const int& _session_idx, const int& _idx_in_graph)
    {
        return (_idx_in_graph - 1) / (_session_idx * kSessionStartIdxOffset);
    } // ungenGlobalNodeIdx


    /**
     * @brief Generates the global node index based on the session index and node offset.
     *
     * Depending on the session index, this function calculates the node index after offset (kSessionStartIdxOffset).
     * The first session is in the range [1, 1 million] indices, the second is between [1 million + 1, 2 million].
     *
     * @param _session_idx The index of the session.
     * @param _node_offset The offset value for the node.
     * @return The calculated global node index.
     *
     * @note The +1 is added to ensure that the 0000 is the anchor node of each session.
     */
    int genGlobalNodeIdx (const int& _session_idx, const int& _node_offset)
    {
        // return (_session_idx * Slam2ref::kSessionStartIdxOffset) + _node_offset + 1;
        // The +1 is necessary so that the 0000 is the anchor node!
        return (_session_idx * kSessionStartIdxOffset) + _node_offset + 1;
    } // genGlobalNodeIdx

    int genAnchorNodeIdx (const int& _session_idx)
    {
        return (_session_idx * kSessionStartIdxOffset); // the anchore node is the first node added for the session in the graph,
        // however, it has 0,0,0 coordinates see first lines of Slam2ref.cpp
        // poseOrigin(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0)))
    } // genAnchorNodeIdx

} // namespace Slam3refParam


inline float rad2deg(float radians)
{ 
    return radians * 180.0 / M_PI; 
}

inline float deg2rad(float degrees)
{ 
    return degrees * M_PI / 180.0; 
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f pclPointToAffine3fOnlyTranslation(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,0, 0, 0);
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

void fsmkdir(std::string _path)
{
    if (!fs::is_directory(_path) || !fs::exists(_path)) 
        fs::create_directories(_path); // create src folder
} //fsmkdir


/**
 * @brief Transforms a point cloud using a 6-DOF pose transformation.
 *
 * This function applies a 6-DOF pose transformation to each point in the input point cloud.
 *
 * @param cloudIn [in] The input point cloud to be transformed.
 * @param transformIn [in] The 6-DOF pose transformation to be applied.
 * @return A pointer to the transformed point cloud.
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    // Create a new point cloud to store the transformed points
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    // Pointer to access individual points in the input cloud
    PointType *pointFrom;

    // Get the size of the input cloud
    int cloudSize = cloudIn->size();

    // Resize the output cloud to the same size as the input cloud
    cloudOut->resize(cloudSize);

    // Create an affine transformation matrix from the input pose
    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z,
                                                      transformIn->roll, transformIn->pitch, transformIn->yaw);

    // Number of CPU cores for parallel processing (TODO: Move to YAML configuration)
    int numberOfCores = 8;

    // Parallel loop for transforming each point in the input cloud

//#pragma omp parallel for num_threads(numberOfCores)
#pragma omp parallel for num_threads(numberOfCores) default(none) shared(cloudIn, cloudOut, transCur, cloudSize) private(pointFrom)
    for (int i = 0; i < cloudSize; ++i)
    {
        // Access the current point in the input cloud
        pointFrom = &cloudIn->points[i];

        // Apply the transformation to the current point and store it in the output cloud
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
//        cloudOut->points[i].intensity = pointFrom->intensity;
    }

    // Return the transformed point cloud
    return cloudOut;
}

pcl::PointCloud<PointTypeRGB>::Ptr transformPointCloud(pcl::PointCloud<PointTypeRGB>::Ptr cloudIn, PointTypePose* transformIn)
{
    // Create a new point cloud to store the transformed points
    pcl::PointCloud<PointTypeRGB>::Ptr cloudOut(new pcl::PointCloud<PointTypeRGB>());

    // Pointer to access individual points in the input cloud
    PointTypeRGB *pointFrom;

    // Get the size of the input cloud
    int cloudSize = cloudIn->size();

    // Resize the output cloud to the same size as the input cloud
    cloudOut->resize(cloudSize);

    // Create an affine transformation matrix from the input pose
    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z,
                                                      transformIn->roll, transformIn->pitch, transformIn->yaw);

    // Number of CPU cores for parallel processing (TODO: Move to YAML configuration)
    int numberOfCores = 8;

    // Parallel loop for transforming each point in the input cloud

//#pragma omp parallel for num_threads(numberOfCores)
#pragma omp parallel for num_threads(numberOfCores) default(none) shared(cloudIn, cloudOut, transCur, cloudSize) private(pointFrom)
    for (int i = 0; i < cloudSize; ++i)
    {
        // Access the current point in the input cloud
        pointFrom = &cloudIn->points[i];

        // Apply the transformation to the current point and store it in the output cloud
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
//        cloudOut->points[i].intensity = pointFrom->intensity;
    }

    // Return the transformed point cloud
    return cloudOut;
}
std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr) 
{ 
    std::vector<std::pair<double, int> > vp; 
    for (int i = 0; i < static_cast<int>(arr.size()); ++i)
        vp.push_back(std::make_pair(arr[i], i)); 
  
    std::sort(vp.begin(), vp.end(), std::greater<>()); 
    return vp;
} 
//
//sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
//{
//    sensor_msgs::PointCloud2 tempCloud;
//    pcl::toROSMsg(*thisCloud, tempCloud);
//    tempCloud.header.stamp = thisStamp;
//    tempCloud.header.frame_id = thisFrame;
//    if (thisPub->getNumSubscribers() != 0)
//        thisPub->publish(tempCloud);
//    return tempCloud;
//}


std::vector<double> splitPoseLine(const std::string& _str_line, char _delimiter) {
    std::vector<double> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter)) {
        parsed.push_back(std::stod(temp)); // convert string to "double"
    }
    return parsed;
}

G2oLineInfo splitG2oFileLine(const std::string& _str_line) {

    std::stringstream ss(_str_line);

	std::vector<std::string> parsed_elms ;
    std::string elm;
	char delimiter = ' ';
    while (getline(ss, elm, delimiter)) {
        parsed_elms.push_back(elm);
    }

	G2oLineInfo parsed;
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kVertexTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.curr_idx = std::stoi(parsed_elms.at(1)); // convert string to "signed integer"
		parsed.trans.push_back(std::stod(parsed_elms.at(2))); // convert string to "double"
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.quat.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
	}
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kEdgeTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.prev_idx = std::stoi(parsed_elms.at(1));
		parsed.curr_idx = std::stoi(parsed_elms.at(2));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.trans.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
		parsed.quat.push_back(std::stod(parsed_elms.at(9)));
	}

	return parsed;
}

bool isTwoStringSame(const std::string& _str1, const std::string _str2)
{
	return !(_str1.compare(_str2));
}

void collect_digits(std::vector<int>& digits, int num) {
    if (num > 9) {
        collect_digits(digits, num / 10);
    }
    digits.push_back(num % 10);
}

void writePose3ToStream(std::fstream& _stream, const gtsam::Pose3& _pose)
{
    gtsam::Point3 t = _pose.translation();
    gtsam::Rot3 R = _pose.rotation();
    // The next is the same as the kitti dataset pose format (as explained in evo): https://github.com/MichaelGrupp/evo/wiki/Formats#kitti---kitti-dataset-pose-format
    // r1 means column 1 (see https://gtsam.org/doxygen/a02759.html)
    std::string sep = " "; // separator
    _stream << R.r1().x() << sep << R.r2().x() << sep << R.r3().x() << sep << t.x() << sep 
            << R.r1().y() << sep << R.r2().y() << sep << R.r3().y() << sep << t.y() << sep
            << R.r1().z() << sep << R.r2().z() << sep << R.r3().z() << sep << t.z() << std::endl;
}


void writePose3ToStream_CC(std::fstream& _stream, const gtsam::Pose3& _pose)
{
    gtsam::Point3 t = _pose.translation();

    // r1 means column 1 (see https://gtsam.org/doxygen/a02759.html)
    std::string sep = " "; // separator
    _stream << t.x() << sep << t.y() << sep << t.z()<< std::endl;
}

void writePose3ToStream_CC_withColor(std::fstream& _stream, const gtsam::Pose3& _pose, std::vector<int> color)
{
    gtsam::Point3 t = _pose.translation();

    // r1 means column 1 (see https://gtsam.org/doxygen/a02759.html)
    std::string sep = " "; // separator
    _stream << t.x() << sep << t.y() << sep << t.z()<< sep << color[0] << sep << color[1] << sep << color[2] << std::endl;
}
// Define the color mapping.
std::map<int, std::vector<int>> colorMap = {
        {1, {0, 255, 0}},   // green
        {2, {0, 0, 255}},   // blue
        {3, {255, 0, 0}},   // red
        {4, {0, 0, 0}}      // black
};


void writeAllPosesWithColorsForCC(std::fstream& _stream, const std::vector<std::pair<int, gtsam::Pose3>>& poseVector, const std::vector<std::pair<int, int>>& poseCategoryVector)
{
    for (const auto& poseCategory : poseCategoryVector)
    {
        int poseIndex = poseCategory.first;
        int category = poseCategory.second;

        // Retrieve the corresponding pose
        const gtsam::Pose3& pose = poseVector[poseIndex].second;

        // Get the color for the current category
        const std::vector<int>& color = colorMap[category];

        writePose3ToStream_CC_withColor(_stream, pose, color);

    }
}

std::vector<int> linspace(int a, int b, int N) {
    int h = (b - a) / static_cast<int>(N-1);
    std::vector<int> xs(N);
    typename std::vector<int>::iterator x;
    int val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

Eigen::MatrixXd readSCD(std::string fileToOpen)
{
	// ref: https://aleksandarhaber.com/eigen-matrix-library-c-tutorial-saving-and-loading-data-in-from-a-csv-file/
	using namespace Eigen;

    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileToOpen);
    std::string matrixRowString;
	std::string matrixEntry;

    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
        while (getline(matrixRowStringStream, matrixEntry, ' ')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries

        matrixRowNumber++; //update the column numbers
    }
    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2)
{
    auto p1x = p1.translation().x();
    auto p1y = p1.translation().y();
    auto p1z = p1.translation().z();
    auto p2x = p2.translation().x();
    auto p2y = p2.translation().y();
    auto p2z = p2.translation().z();
    
    return sqrt((p1x-p2x)*(p1x-p2x) + (p1y-p2y)*(p1y-p2y) + (p1z-p2z)*(p1z-p2z));
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector3i& color)
{
    // Define a new point cloud for the colored points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Go through each point in the input point cloud
    for(const auto& point : cloud->points)
    {
        // Create a new colored point
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;

        // Assign color to the point
        uint8_t r = color(0), g = color(1), b = color(2);
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        float f_rgb;
        std::memcpy(&f_rgb, &rgb, sizeof(float));
        colored_point.rgb = f_rgb;

        // Add the colored point to the colored cloud
        colored_cloud->push_back(colored_point);
    }

    return colored_cloud;
}


void colorOpen3DPointCloud(open3d::geometry::PointCloud& open3d_cloud, const Eigen::Vector3i& color)
{

    open3d_cloud.colors_.resize(open3d_cloud.points_.size());

    // Normalize the color to float
    Eigen::Vector3d normalized_color = color.cast<double>() / 255.0;

    // Go through each point in the input point cloud
    for(size_t i = 0; i < open3d_cloud.points_.size(); i++)
    {
        // Assign color to the point
        open3d_cloud.colors_[i] = normalized_color;
    }
}


/**
 * @brief Filters a point cloud based on the radius range of a specific axis.
 *
 * This function applies a radius filter on a given point cloud in the xy-plane,
 * to keep only the points with radius values within the specified range.
 * The resulting filtered point cloud is returned.
 *
 * @param cloud The input point cloud to be filtered.
 * @param minRadius The minimum radius to be included in the filtered point cloud.
 * @param maxRadius The maximum radius to be included in the filtered point cloud.
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr A shared pointer to the filtered point cloud.
 *
 * @see pcl::PointCloud
 * @see pcl::PointXYZ
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr radiusFilterPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        double minRadius,
        double maxRadius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : cloud->points) {
        double radius = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));

        if (radius >= minRadius && radius <= maxRadius) {
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;
    filteredCloud->is_dense = true;

    return filteredCloud;
}


pcl::PointCloud<PointType>::Ptr filterPointsWithGrid(
        pcl::PointCloud<PointType>::Ptr inputCloud,
        const double gridSize,
        const size_t minPointsInGrid)
{

    typedef std::pair<int, int> GridIndex;
    std::map<GridIndex, std::vector<PointType>> grid;

    for (const auto& point : inputCloud->points) {
        int i = static_cast<int>(point.x / gridSize);
        int j = static_cast<int>(point.y / gridSize);

        grid[GridIndex(i, j)].push_back(point);
    }

    pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>());

    for (const auto& cell : grid) {
        if (cell.second.size() >= minPointsInGrid) {
            for (const auto& point : cell.second) {
                filteredCloud->points.push_back(point);
            }
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;
    filteredCloud->is_dense = true;

    return filteredCloud;
}



void rotatePointCloud(pcl::PointCloud<PointType>::Ptr cloud, float yaw_angle) {
    if (cloud->empty()) {
        std::cerr << "Error: Point cloud is empty. Cannot perform rotation." << std::endl;
        return;
    }

    // Define the rotation matrix using Eigen
    Eigen::Affine3f rotation_matrix = Eigen::Affine3f::Identity();
    rotation_matrix.rotate(Eigen::AngleAxisf(yaw_angle, Eigen::Vector3f::UnitZ()));

    // Apply the rotation to the point cloud
    pcl::transformPointCloud(*cloud, *cloud, rotation_matrix);
}



void pclToOpen3d(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::shared_ptr<open3d::geometry::PointCloud> open3d_cloud) {
    open3d_cloud->Clear();
    open3d_cloud->points_.reserve(pcl_cloud->points.size());

    for (const auto& point : pcl_cloud->points) {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
            open3d_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
    }
}


std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3d_v2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();
    open3d_cloud->points_.reserve(pcl_cloud->points.size());

    for (const auto& point : pcl_cloud->points) {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
            open3d_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
    }

    return open3d_cloud;
}

void open3dToPcl(const std::shared_ptr<open3d::geometry::PointCloud>& open3d_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {
    pcl_cloud->points.reserve(open3d_cloud->points_.size());

    for (const auto& point : open3d_cloud->points_) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(point(0));
        pcl_point.y = static_cast<float>(point(1));
        pcl_point.z = static_cast<float>(point(2));

        pcl_cloud->points.push_back(pcl_point);
    }

    pcl_cloud->width = pcl_cloud->points.size();  // Number of points in the cloud
    pcl_cloud->height = 1;  // Indicates it is organized
}


// Define the transformation function from PointTypePose to pcl::PointXYZ
pcl::PointXYZ transformPoint(const PointTypePose& pose) {
    pcl::PointXYZ point;
    point.x = pose.x;
    point.y = pose.y;
    point.z = pose.z;
    return point;
}

// Function to convert PointCloud<PointTypePose> to PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud(const pcl::PointCloud<PointTypePose>::Ptr& inputCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    outputCloud->points.resize(inputCloud->points.size());

    for (size_t i = 0; i < inputCloud->points.size(); ++i) {
        outputCloud->points[i] = transformPoint(inputCloud->points[i]);
    }

    return outputCloud;
}


std::vector<int> findKNearestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudKeyPoses6D,
                                    const pcl::PointXYZ& inputQueryPoint,
                                    int k) {
    // Create a nanoflann k-d tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudKeyPoses6D);

    // Perform a KNN search
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);

    if (kdtree.nearestKSearch(inputQueryPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        return pointIdxNKNSearch;
    } else {
        std::cerr << "No nearest neighbors found." << std::endl;
        return std::vector<int>(); // Return an empty vector to indicate no neighbors found
    }
}



// Function that gets pose transformation
gtsam::Pose3 getPoseTransformation(const Eigen::Matrix4d& resultMatrix)
{
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame = static_cast<Eigen::Affine3f>(resultMatrix.cast<float>());

    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

    auto result = poseTo.between(poseFrom);

    return result;
}

void loadPointCloud(const std::string &filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // Get the extension of the file
    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);

    if (extension == "pcd")
    {
        // Load PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read the .pcd file %s\n", filePath.c_str());
            return;
        }
        else
        {
            std::cout << "Successfully loaded " << filePath << " as a .pcd file" << std::endl;
        }
    }
    else if (extension == "ply")
    {
        // Load PLY file
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read the .ply file %s\n", filePath.c_str());
            return;
        }
        else
        {
            std::cout << "Successfully loaded " << filePath << " as a .ply file" << std::endl;
        }
    }
    else
    {
        PCL_ERROR("Unsupported file format: %s\n", extension.c_str());
    }
}


void downsamplePointCloud(pcl::PointCloud<PointType>::Ptr& originalCloud, float leaf_size)
{
    // Create a VoxelGrid object
    pcl::VoxelGrid<PointType> voxel_grid_filter;

    // Set the input cloud to the filter
    voxel_grid_filter.setInputCloud(originalCloud);

    // Set the voxel grid leaf size
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    // Call the filtering function to obtain the downsampled output
    voxel_grid_filter.filter(*originalCloud);
}



pcl::PointCloud<PointType>::Ptr cropPointCloudWithinSphere(
        const pcl::PointCloud<PointType>::Ptr& target_cloud,
        const Eigen::Vector3f& center,
        const double sphere_radius, const int numberOfCores
) {
//    size_t numberOfCores = omp_get_max_threads();
    std::vector<pcl::PointCloud<PointType>> sphere_points(numberOfCores);
    size_t num_points_per_thread = target_cloud->points.size() / numberOfCores;

#pragma omp parallel for num_threads(numberOfCores) default(none) shared(numberOfCores, target_cloud, center, sphere_radius, sphere_points, num_points_per_thread)
    for (int i = 0; i < numberOfCores; ++i) {
        auto start_iter = std::next(target_cloud->points.begin(), i * num_points_per_thread);
        auto end_iter = i == numberOfCores - 1 ? target_cloud->points.end() : std::next(start_iter, num_points_per_thread);

        for (auto it = start_iter; it != end_iter; ++it) {
            Eigen::Vector3f point_pos(it->x, it->y, it->z);
            if ((point_pos - center).norm() < sphere_radius) {
                sphere_points[i].points.push_back(*it);
            }
        }
    }

    pcl::PointCloud<PointType>::Ptr sphere_cloud(new pcl::PointCloud<PointType>());
    for (const auto& subv : sphere_points) {
        sphere_cloud->points.insert(sphere_cloud->points.end(), subv.points.begin(), subv.points.end());
    }

    sphere_cloud->width = sphere_cloud->points.size();
    sphere_cloud->height = 1;
    sphere_cloud->is_dense = false;

    return sphere_cloud;
}


void copyFile(std::string sourcePath, std::string destinationDir) {
    try {
        boost::filesystem::path source(sourcePath);
        boost::filesystem::path destination(destinationDir);

        // Adding the source file name to the destination directory
        destination /= source.filename();

        boost::filesystem::copy_file(source, destination, boost::filesystem::copy_option::overwrite_if_exists);

        // Print filename to console
        std::cout << "  File " << source.filename().string() << " copied successfully\n";
    }
    catch (boost::filesystem::filesystem_error const & e) {
        std::cerr << e.what() << '\n';
    }
}



// Custom logging function using printf-style formatting
void logInfo_print(const char* format, ...) {
    va_list args;           // Define a variable argument list
    va_start(args, format); // Initialize the variable argument list with the format string
    std::vprintf(format, args); // Print formatted data from the variable argument list
    va_end(args);           // Clean up the variable argument list
}

