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
//#include "Eigen/Dense"

//Open3D


PlaneDetection::PlaneDetection()
{

}

PlaneDetection::~PlaneDetection()
{

}
double getAngleTwoVectors(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2) {
    double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    return radian_angle;   //[0,PI]
}
bool PlaneDetection::readMeshFile(string filename)
{
    clock_t startTime,endTime;
    startTime = clock();
    int vertex_idx = 0;
    int maxIdentifyPoints = 1200;
    std::vector< std::vector<int> > membership;
    auto cloud_ptr = std::make_shared<geometry::PointCloud>();
    std::random_device rd;
    std::mt19937 rng(rd());
    auto normalOutPlane = std::shared_ptr<open3d::geometry::PointCloud>();
    auto normalInPlane = std::shared_ptr<open3d::geometry::PointCloud>();
    auto exceptPlane = std::shared_ptr<open3d::geometry::PointCloud>();
//    vector<shared_ptr<open3d::geometry::PointCloud>> normalPlaneStore;
//    auto outPlane = std::shared_ptr<open3d::geometry::PointCloud>();
//    auto normal = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(filename, *cloud_ptr)) {
        utility::LogInfo("Successfully read {}", filename);
    } else {
        utility::LogWarning("Failed to read {}", filename);
        return 1;
    }
//    cloud_ptr->NormalizeNormals();
    startTime = clock();

//    cloud_ptr = std::get<0>(cloud_ptr->RemoveRadiusOutliers(20,0.2));
    cloud_ptr = cloud_ptr->UniformDownSample(15);
//    cloud_ptr = cloud_ptr->VoxelDownSample(0.1);
//    cloud_ptr = std::get<0>(cloud_ptr->RemoveRadiusOutliers(5,0.2));
    cloud_ptr = std::get<0>(cloud_ptr->RemoveStatisticalOutliers(20,1.5));
    visualization::Visualizer visualizer;
    int i = 0;
    int qualifiedPlane = 0;
    int planeCount = 0;
    int maxIteration = 5; // 5
    int maxLoopIteration = 15; // 15
    int normalInliersThreshold = 1000;
    int normalSelectIndexThreshold = 500;
    double deltaAngleThreshold = 0.002;
    std::vector<Eigen::Vector3d> vColorInPlane; // = {0, 0, 255};
    std::vector<size_t> normalInliers;
    vector<Eigen::Matrix<double, 3, 1>> normalVector;
    std::vector<Eigen::Matrix<double, 4, 1>> paramaterPlaneVector;
//    Eigen::Matrix<double, 4, 1> parametersPlane;
//    while (std::get<1>(cloud_ptr->SegmentPlane(0.05,3,100)).size() >= maxIdentifyPoints)
//    while (std::get<1>(cloud_ptr->SegmentPlane(0.05,3,100)).size() >= maxIdentifyPoints)
    for(int w = 0; w < maxLoopIteration; w++)
    {
        i++;
        int color_index = i%6;
        cloud_ptr->NormalizeNormals();
        cloud_ptr->EstimateNormals(geometry::KDTreeSearchParamHybrid(0.008,25));
        auto base = cloud_ptr->normals_[rng()%cloud_ptr->normals_.size()];
//        auto base = cloud_ptr->normals_[0];
        for (size_t idx = 0; idx < cloud_ptr->normals_.size(); ++idx) {
            double resultAngle = getAngleTwoVectors(base,cloud_ptr->normals_[idx]);
//            cout << "result Angle" << "               " << resultAngle << endl;
            if (resultAngle <= deltaAngleThreshold)
            {
//                cout << "enter here" << endl;
                normalInliers.emplace_back(idx);
                normalVector.emplace_back(cloud_ptr->normals_[idx]);
//                cloud_ptr->points_.erase(cloud_ptr->points_.begin()+idx-1);
            }
        }
        for (int ite = 0; ite < maxIteration; ++ite) {
            auto baseIte = normalVector[rng()%normalVector.size()];
            for (int m = 0; m < cloud_ptr->normals_.size(); ++m) {
                if (getAngleTwoVectors(baseIte,cloud_ptr->normals_[m]) <= deltaAngleThreshold)
                {
//                cout << "enter here" << endl;
                    normalInliers.emplace_back(m);
                    normalVector.emplace_back(cloud_ptr->normals_[m]);
//                    cloud_ptr->points_.erase(cloud_ptr->points_.begin()+m-1);
                }
            }
        }
//        normal->points_.assign(normalVector.begin(),normalVector.end());
        cout << "normalInliersSize" << "     " << normalInliers.size() << endl;
//        cout << "normalPoints size" << "     " << normal->points_.size() << endl;
        if (normalInliers.size() <= normalInliersThreshold)
        {
            continue;
        }
        normalInPlane = cloud_ptr->SelectByIndex(normalInliers, false);
        normalOutPlane = cloud_ptr->SelectByIndex(normalInliers, true);
        std::tuple<Eigen::Vector4d, std::vector<size_t>> Result = normalInPlane->SegmentPlane(0.05,3,100);
//        std::tuple<Eigen::Vector4d, std::vector<size_t>> ResultNormal = normal->SegmentPlane(1,3,100);
//        std::tuple<Eigen::Vector4d, std::vector<size_t>> Result = cloud_ptr->SegmentPlane(0.1, 300, 100);
//        Eigen::Vector4d parameter = std::get<0>(Result);
//        std::vector<size_t> normalIndex = std::get<1>(ResultNormal);
//        std::shared_ptr<open3d::geometry::PointCloud> normalinPlane = cloud_ptr->SelectByIndex(normalIndex, false);
        std::vector<size_t> normalSelectIndex = std::get<1>(Result);
        if (normalSelectIndex.size() < normalSelectIndexThreshold)
        {
            continue;
        }
        planeCount++;
        Eigen::Vector4d parametersPlane = std::get<0>(Result);
        cout << "parameter of the detect plane" << parametersPlane << endl;
        paramaterPlaneVector.emplace_back(parametersPlane);
        cout << "size of the plane parameter vector" << paramaterPlaneVector.size() << endl;
        cout << "normalSelectIndex" << " " << normalSelectIndex.size() << endl;
        cout << "This is the" << planeCount << "th plane match the need" << endl;
        exceptPlane = normalInPlane->SelectByIndex(normalSelectIndex, true);
        normalInPlane = normalInPlane->SelectByIndex(normalSelectIndex, false);
//        auto exceptNormalInPlane = normalInPlane->SelectByIndex(normalSelectIndex, true);
        *normalOutPlane += *exceptPlane;
//        normalPlaneStore.push_back(normalInPlane);
        qualifiedPlane ++ ;
        cout << "stop at here" << endl;
        Eigen::Vector3d c = { 0,0,255 };
        Eigen::Vector3d d = {255,0,0};
//        normal->PaintUniformColor(c);
//        normalInPlane->PaintUniformColor(c);
//        normalOutPlane->PaintUniformColor(d);
//        visualization::DrawGeometries({cloud_ptr},"plane",1600,900,50,50, true);
//        visualization::DrawGeometries({normal},"plane after iteration",1600,900);
        // save ply files
//        open3d::io::WritePointCloud(to_string(i)+"mesh_.ply", *inPlane);
//
//        // add color to each plane region
        double r = xjRandom(0, 200);
        double g = xjRandom(100, 256/2);
        double b = xjRandom(50/2, 250);
        r /= 255.0;
        g /= 255.0;
        b /= 255.0;
        Eigen::Vector3d e = { r,g,b };
        normalInPlane->PaintUniformColor(e);
        const Eigen::Vector3d colorOutPlane = {255, 0, 0};
        normalOutPlane->PaintUniformColor(colorOutPlane);
//
//        outPlane = cloud_ptr->SelectByIndex(selectIndex, true);
//        const Eigen::Vector3d colorOutPlane = {255, 0, 0};
//        outPlane->PaintUniformColor(colorOutPlane);
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
        visualizer.AddGeometry(normalInPlane);
        cloud_ptr = normalOutPlane;
        normalInliers.clear();
        normalVector.clear();
//        default_delete<shared_ptr<geometry::PointCloud>> normalOutPlane;
//        cloud_ptr = outPlane;
    }
    endTime = clock();//计时结束
    cout << "The run time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    cout << "Total good plane" << "  " << planeCount << endl;
    visualizer.AddGeometry(normalOutPlane);
    visualizer.Run();
//    paramaterPlaneVector.clear();
//    normalPlaneStore.clear();

    return true;
}


