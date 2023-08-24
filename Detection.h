#pragma once

#include "open3d/Open3D.h"

struct Plane {
	Eigen::Vector3d center;
	Eigen::Vector4d equation;
	std::vector<size_t> point_index;
};

static class PlaneSegmentation {
public:
	std::tuple<Eigen::Vector4d, std::vector<size_t>> static PlaneRegression(open3d::geometry::PointCloud& pcd, float threshold = 0.01, int init_n = 3, int iter = 1000);
	std::vector<Plane> static DetectMultiPlanes(open3d::geometry::PointCloud& pcd, float min_ratio = 0.05, float threshold = 0.01, int iterations = 1000);
	std::pair<open3d::geometry::PointCloud, Eigen::Vector3d> static RemoveGivenPointSet(open3d::geometry::PointCloud& pcd, std::vector<size_t> index_set);
};