## 2021.07.21
### Finished
1. Finish add the Cross-Point and Cross-Line on the Map class, the extraction of the points and lines take place in Track(), located in
Tracking.cc. When the number of the planes in the image = 2, the Track() will execute DetectCrossLine() tp compute the CrossLine and 
store it on the vector with PairPlanes ID. When the number of the planes in the image >= 3, the Track() will execute the DetectCrossPoint() and
store the CrossPoint on the vector with TuplePlanes ID. Three planes must qualify the condition that at least 2 planes are perpendicular
with the third plane, if not, the DetectCrossPoint() will execute extracting CrossLine from 2 planes and store them
### TODO
1. 新的图像帧进来, 需要建立提取的平面和地图中的landmrks之间的关系 need to be accomplished  
# ManhattanSLAM

# 1.Prerequisites
We have tested the library in **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## PCL
We use [PCL](http://www.pointclouds.org/) to reconstruct and visualize mesh. Download and install instructions can be found at: https://github.com/ros-perception/perception_pcl. **Tested with PCL 1.7.0**.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 2. Test the system
