//
// Created by AlexLiu on 2021/6/3.
//
#include "plane_detection.h"
#include <stdint.h>
#include <string>
#include <iomanip>

#include <iostream>
#include <memory>
#include <random>
#include <cstdlib>
#include <ctime>
#define xjRandom(a,b) (rand()%(b-a)+a)

//Eigen
#include "Eigen/Dense"

//Open3D


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
    int i = 0;
    std::vector<Eigen::Vector3d> vColorInPlane; // = {0, 0, 255};

    while (std::get<1>(cloud_ptr->SegmentPlane(0.05,3,100)).size() >= maxIdentifyPoints)
    {
        i++;
        int color_index = i%6;
        std::tuple<Eigen::Vector4d, std::vector<size_t>> Result = cloud_ptr->SegmentPlane(0.1, 300, 100);
        Eigen::Vector4d parameter = std::get<0>(Result);

        std::vector<size_t> selectIndex = std::get<1>(Result);
        std::shared_ptr<open3d::geometry::PointCloud> inPlane = cloud_ptr->SelectByIndex(selectIndex, false);

        // save ply files
        open3d::io::WritePointCloud(to_string(i)+"mesh_.ply", *inPlane);

        // add color to each plane region
        double r = xjRandom(0, 200);
        double g = xjRandom(100, 256/2);
        double b = xjRandom(50/2, 250);
        r /= 255.0;
        g /= 255.0;
        b /= 255.0;
        Eigen::Vector3d c = { r,g,b };
        inPlane->PaintUniformColor(c);

        outPlane = cloud_ptr->SelectByIndex(selectIndex, true);
        const Eigen::Vector3d colorOutPlane = {255, 0, 0};
        outPlane->PaintUniformColor(colorOutPlane);
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
        visualizer.AddGeometry(inPlane);
        cloud_ptr = outPlane;
    }
    visualizer.AddGeometry(outPlane);
    visualizer.Run();
    return true;
}


