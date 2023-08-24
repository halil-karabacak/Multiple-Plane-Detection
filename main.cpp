#include <iostream>

#include "open3d/Open3D.h"
#include "Detection.h"

int main() {
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug); // Set verbosity level

    std::string file_path = "fragment.ply";
    auto point_cloud = open3d::io::CreatePointCloudFromFile(file_path);
    
    // pre - processing
    point_cloud->RemoveNonFinitePoints();
    point_cloud->RemoveStatisticalOutliers(25, 0.2);
    point_cloud = point_cloud->VoxelDownSample(0.01);
    
    auto planes = PlaneSegmentation::DetectMultiPlanes(*point_cloud, 0.1, 0.005, 500);


    return 0;
}
