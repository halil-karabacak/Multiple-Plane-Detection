#include <iostream>

#include "open3d/Open3D.h"
#include "Detection.h"

int main() {
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug); // Set verbosity level

    std::string file_path = "../data/test2.ply";
    auto point_cloud = open3d::io::CreatePointCloudFromFile(file_path);
    
    // preprocessing
    open3d::geometry::PointCloud res = point_cloud->RemoveNonFinitePoints();
    std::shared_ptr<open3d::geometry::PointCloud > pcd =  std::get<0>(res.RemoveStatisticalOutliers(50, 0.5));

    auto start = std::chrono::high_resolution_clock::now();
    auto planes = PlaneSegmentation::DetectMultiPlanes(*pcd, 0.05, 0.005, 2000);
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << planes.size() << " planes detected!" << std::endl;

    std::chrono::duration<double> duration = end - start;

    std::cout << "Multi plane detection took: " << duration.count() << " seconds" << std::endl;

}
