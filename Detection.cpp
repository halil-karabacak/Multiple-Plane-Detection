#include "Detection.h"

std::tuple<Eigen::Vector4d, std::vector<size_t>> PlaneSegmentation::PlaneRegression(open3d::geometry::PointCloud& pcd, float threshold, int init_n, int iter)
{
	return pcd.SegmentPlane(threshold, init_n, iter);
}

std::vector<Plane> PlaneSegmentation::DetectMultiPlanes(open3d::geometry::PointCloud& pcd, float min_ratio, float threshold, int iterations)
{
	std::vector<Plane> plane_list;
	int size = (int)pcd.points_.size();
	auto target = pcd;

	int count = 0;
	int t = 0;

	std::cout << "Girdi \n";

	while (count < (1 - min_ratio) * size) {
		std::cout << t << "\n";
		std::tuple<Eigen::Vector4d, std::vector<size_t>> plane = PlaneRegression(target, threshold=threshold, 3, iterations);
		Plane planeObj;
		Eigen::Vector4d plane_eq = std::get<0>(plane);
		std::vector<size_t> index = std::get<1>(plane);

		planeObj.equation = plane_eq;
		planeObj.point_index = index;
		// planeObj.center =
		
		count += (int)index.size();
		t++;

		auto res = RemoveGivenPointSet(target, index);
		target = res.first;
		planeObj.center = res.second;

		plane_list.push_back(planeObj);

		open3d::io::WritePointCloudToPLY("output_" + std::to_string(t) + ".ply", target, open3d::io::WritePointCloudOption());
	}
	return plane_list;
}


std::pair<open3d::geometry::PointCloud, Eigen::Vector3d> PlaneSegmentation::RemoveGivenPointSet(open3d::geometry::PointCloud& pcd, std::vector<size_t> index_set)
{
	std::vector<Eigen::Vector3d> tmp_points;
	std::vector<Eigen::Vector3d> tmp_normals;
	std::vector<Eigen::Vector3d> tmp_colors;
	std::vector<Eigen::Matrix3d> tmp_covariances;

	double avg_x = 0, avg_y = 0, avg_z = 0;
	int sz = 0;


	
	for (size_t i = 0; i < pcd.points_.size(); i++) {
		if (std::find(index_set.begin(), index_set.end(), i) != index_set.end()) {
			tmp_points.push_back(pcd.points_[i]);
			if (pcd.HasNormals())
				tmp_normals.push_back(pcd.normals_[i]);
			if (pcd.HasColors())
				tmp_colors.push_back(pcd.colors_[i]);
			if (pcd.HasCovariances())
				tmp_covariances.push_back(pcd.covariances_[i]);
		}
		else {
			sz++;
			avg_x += pcd.points_[i].x();
			avg_y += pcd.points_[i].y();
			avg_z += pcd.points_[i].z();
		}
	}

	pcd.points_ = tmp_points;
	pcd.normals_ = tmp_normals;
	pcd.colors_ = tmp_colors;
	pcd.covariances_ = tmp_covariances;

	return std::make_pair(pcd, Eigen::Vector3d(avg_x / sz, avg_y / sz, avg_z / sz));
}
