#include "open3d/Open3D.h"

// Define a struct to represent a plane in 3D space
struct Plane {
    Eigen::Vector3d center;                     // Center of the plane
    Eigen::Vector4d equation;                   // Equation of the plane (ax + by + cz + d = 0)
    std::vector<Eigen::Vector3d> point_coord;   // Coordinates of points in the plane
};

// Define a static class for plane segmentation operations
class PlaneSegmentation {
public:
    // Perform plane regression using RANSAC algorithm
    // Parameters:
    // - pcd: Point cloud to be segmented
    // - threshold: Distance threshold for plane fitting
    // - init_n: Initial number of points for fitting a plane
    // - iter: Number of RANSAC iterations
    // Returns a tuple containing the fitted plane equation and inlier indices
    static std::tuple<Eigen::Vector4d, std::vector<size_t>> PlaneRegression(
        open3d::geometry::PointCloud& pcd, float threshold = 0.01, int init_n = 3, int iter = 1000);

    // Detect multiple planes in a point cloud using plane segmentation
    // Parameters:
    // - pcd: Point cloud to be segmented
    // - min_ratio: Minimum ratio of inliers for a plane to be considered
    // - threshold: Distance threshold for plane fitting
    // - iterations: Number of segmentation iterations
    // - upper_limit: number that represent the expected value of upper limit of planes 
    // Returns a vector of detected planes
    static std::vector<Plane> DetectMultiPlanes(
        const open3d::geometry::PointCloud& pcd, float min_ratio = 0.05, float threshold = 0.01, int iterations = 1000, int upper_limit = 15);

    // Remove a given set of points from a point cloud
    // Parameters:
    // - pcd: Original point cloud
    // - index_set: Indices of points to be removed
    // Returns a pair containing the modified point cloud and information about removed points
    static std::pair<open3d::geometry::PointCloud, std::pair<Eigen::Vector3d, std::vector<Eigen::Vector3d>>> RemoveGivenPointSet(
        const open3d::geometry::PointCloud& pcd, std::vector<size_t> index_set);
};
