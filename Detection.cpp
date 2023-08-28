#include "Detection.h"

std::tuple<Eigen::Vector4d, std::vector<size_t>> PlaneSegmentation::PlaneRegression(open3d::geometry::PointCloud& pcd, float threshold, int init_n, int iter)
{
	return pcd.SegmentPlane(threshold, init_n, iter);
}

std::vector<Plane> PlaneSegmentation::DetectMultiPlanes(const open3d::geometry::PointCloud& pcd, float min_ratio, float threshold, int iterations, int upper_limit)
{
	std::vector<Plane> plane_list;
	int size = (int)pcd.points_.size();
	auto target = pcd;
	int count = 0;
	int plane_count = 0;

	while (count < (1 - min_ratio) * size && plane_count < upper_limit) {
		std::tuple<Eigen::Vector4d, std::vector<size_t>> plane = PlaneRegression(target, threshold=threshold, 3, iterations);
		Plane planeObj;
		Eigen::Vector4d plane_eq = std::get<0>(plane);
		std::vector<size_t> index = std::get<1>(plane);

		count += (int)index.size();
		plane_count++;
		auto res = RemoveGivenPointSet(target, index);

		target = res.first;
		planeObj.center = res.second.first;
		planeObj.point_coord = res.second.second;
		planeObj.equation = plane_eq;
		plane_list.push_back(planeObj);
	}

	return plane_list;
}


std::pair<open3d::geometry::PointCloud, std::pair<Eigen::Vector3d, std::vector<Eigen::Vector3d>>> PlaneSegmentation::RemoveGivenPointSet(const open3d::geometry::PointCloud& pcd, std::vector<size_t> index_set)
{
	std::vector<Eigen::Vector3d> tmp_points; tmp_points.reserve(pcd.points_.size() - index_set.size());
	std::vector<Eigen::Vector3d> tmp_normals; tmp_normals.reserve(pcd.points_.size() - index_set.size());
	std::vector<Eigen::Vector3d> tmp_colors; tmp_colors.reserve(pcd.points_.size() - index_set.size());
	std::vector<Eigen::Matrix3d> tmp_covariances; tmp_covariances.reserve(pcd.points_.size() - index_set.size());
	open3d::geometry::PointCloud out;

	std::vector<Eigen::Vector3d> plane_coord;
	if (index_set.size() == 0) {
		std::cout << "Set shouldn't be empty!" << std::endl;
		return std::make_pair(pcd, std::make_pair(Eigen::Vector3d::Identity(), std::vector<Eigen::Vector3d>()));
	}

	double avg_x = 0, avg_y = 0, avg_z = 0;


	for (size_t i = 0; i < pcd.points_.size(); i++) {
		if (std::find(index_set.begin(), index_set.end(), i) == index_set.end()) {
			tmp_points.push_back(pcd.points_[i]);
			if (pcd.HasNormals())
				tmp_normals.push_back(pcd.normals_[i]);
			if (pcd.HasColors())
				tmp_colors.push_back(pcd.colors_[i]);
			if (pcd.HasCovariances())
				tmp_covariances.push_back(pcd.covariances_[i]);
		}
		else {
			plane_coord.push_back(pcd.points_[i]);
			avg_x += pcd.points_[i].x();
			avg_y += pcd.points_[i].y();
			avg_z += pcd.points_[i].z();
		}
	}

	out.points_ = tmp_points;
	out.normals_ = tmp_normals;
	out.colors_ = tmp_colors;
	out.covariances_ = tmp_covariances;

	return std::make_pair(out, std::make_pair(Eigen::Vector3d(avg_x / index_set.size(), avg_y / index_set.size(), avg_z / index_set.size()), plane_coord));
}

