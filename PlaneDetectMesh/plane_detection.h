//
// Created by yons on 2021/6/3.
//

#ifndef PLANEDETECTMESH_PLANE_DETECTION_H
#define PLANEDETECTMESH_PLANE_DETECTION_H
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "open3d/Open3D.h"
#include <iostream>
#include <memory>
#include <thread>
#include "opencv2/opencv.hpp"
#include <string>
#include <fstream>
#include <Eigen/Eigen>
#include "Eigen/Dense"

using namespace std;
using namespace open3d;

class PlaneDetection
{
public:
    PlaneDetection();
    ~PlaneDetection();

    bool readMeshFile(string filename);
};

#endif //PLANEDETECTMESH_PLANE_DETECTION_H
