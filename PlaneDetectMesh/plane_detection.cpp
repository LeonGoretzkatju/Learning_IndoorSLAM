//
// Created by AlexLiu on 2021/6/3.
//
#include "plane_detection.h"
#include <stdint.h>
#include <string>
#include <iomanip>
PlaneDetection::PlaneDetection()
{

}

PlaneDetection::~PlaneDetection()
{

}
bool PlaneDetection::readMeshFile(string filename)
{
    int vertex_idx = 0;
    int maxIdentifyPoints = 1200;
    std::vector< std::vector<int> > membership;
    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(filename, *cloud_ptr)) {
        utility::LogInfo("Successfully read {}", filename);
    } else {
        utility::LogWarning("Failed to read {}", filename);
        return 1;
    }
    cloud_ptr->NormalizeNormals();
    visualization::Visualizer visualizer;
    auto outPlane = std::shared_ptr<open3d::geometry::PointCloud>();
    while (std::get<1>(cloud_ptr->SegmentPlane(0.05,3,100)).size() >= maxIdentifyPoints)
    {
        std::tuple<Eigen::Vector4d, std::vector<size_t>> Result = cloud_ptr->SegmentPlane(0.05, 3, 100);
        Eigen::Vector4d parameter = std::get<0>(Result);
        std::vector<size_t> selectIndex = std::get<1>(Result);
        std::shared_ptr<open3d::geometry::PointCloud> inPlane = cloud_ptr->SelectByIndex(selectIndex, false);
        const Eigen::Vector3d colorInPlane = {0, 0, 255};
        inPlane->PaintUniformColor(colorInPlane);
        outPlane = cloud_ptr->SelectByIndex(selectIndex, true);
        const Eigen::Vector3d colorOutPlane = {0, 0, 0};
        outPlane->PaintUniformColor(colorOutPlane);
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
        visualizer.AddGeometry(inPlane);
        cloud_ptr = outPlane;
    }
    visualizer.AddGeometry(outPlane);
    visualizer.Run();
    return true;
}


