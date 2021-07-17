#include <iostream>
#include "plane_detection.h"

using namespace std;
using namespace open3d;
PlaneDetection plane_detection;
void printUsage()
{
    cout << "How to use:  "<<
         "usage:  ./MeshPlaneDetection your path to the ply file" << endl;
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        printUsage();
        return -1;
    }
    plane_detection.readMeshFile(argv[1]);
    return 0;
}
