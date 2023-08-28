#include <iostream>

#include "open3d/Open3D.h"
#include "Detection.h"

int main() {
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug); // Set verbosity level

    std::string file_path = "../data/test1.ply";
    auto point_cloud = open3d::io::CreatePointCloudFromFile(file_path);
    
    // preprocessing
    open3d::geometry::PointCloud res = point_cloud->RemoveNonFinitePoints();
    std::shared_ptr<open3d::geometry::PointCloud > pcd =  std::get<0>(res.RemoveStatisticalOutliers(50, 1));

    auto start = std::chrono::high_resolution_clock::now();
    
    auto planes = PlaneSegmentation::DetectMultiPlanes(*pcd, 0.05, 0.005, 2000);
    
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << planes.size() << " planes detected!" << std::endl;

    std::chrono::duration<double> duration = end - start;

    std::cout << "Multi plane detection took: " << duration.count() << " seconds" << std::endl;

    // Only for visualization!
    int t = 0;
    for (auto plane : planes) {
        double color = (std::rand() % 255) / 255;
        open3d::geometry::PointCloud a;
        t++;
        double rand = (double)(std::rand() % 255) / 255;
        for (auto ind : plane.point_coord) {
            a.points_.push_back(ind);
            a.colors_.push_back(Eigen::Vector3d(rand, rand, rand));
        }
        open3d::io::WritePointCloudToPLY("Plane_" + std::to_string(t) + ".ply", a, open3d::io::WritePointCloudOption());
    }
}
