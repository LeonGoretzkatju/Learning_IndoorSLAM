/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include <unordered_map>

#include <mutex>


#include "MapLine.h"
#include "SurfelElements.h"

#include "MapPlane.h"
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>

namespace ORB_SLAM2
{

    class MapPoint;
    class KeyFrame;
    class MapLine;
    class MapPlane;
    class Frame;

    struct PartialManhattanMapHash {
        size_t operator() (const std::pair<MapPlane*, MapPlane*>& key) const;
    };

    struct PartialManhattanMapEqual {
        bool operator() (const std::pair<MapPlane*, MapPlane*>& a, const std::pair<MapPlane*, MapPlane*>& b) const;
    };

    struct ManhattanMapHash {
        size_t operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& key) const;
    };

    struct ManhattanMapEqual {
        bool operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& a, const std::tuple<MapPlane*, MapPlane*, MapPlane*>& b) const;
    };

    struct TuplePlaneMapHash {
        size_t operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& key) const;
    };

    struct TuplePlaneMapEqual {
        bool operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& a, const std::tuple<MapPlane*, MapPlane*, MapPlane*>& b) const;
    };

    struct PairPlaneMapEqual {
        bool operator() (const std::pair<MapPlane*, MapPlane*>& a, const std::pair<MapPlane*, MapPlane*>& b) const;
    };

    struct PairPlaneMapHash {
        size_t operator() (const std::pair<MapPlane*, MapPlane*>& key) const;
    };

    class Map
    {
    public:
        cv::Mat CrossPoint;
        cv::Mat CrossLine;
        typedef std::pair<MapPlane*, MapPlane*> PairPlane;
        typedef std::unordered_map<PairPlane, KeyFrame*, PairPlaneMapHash, PairPlaneMapEqual> PairPlanes;
        typedef std::tuple<MapPlane*, MapPlane*, MapPlane*> TuplePlane;
        typedef std::unordered_map<TuplePlane, KeyFrame*, TuplePlaneMapHash, TuplePlaneMapEqual> TuplePlanes;

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud <PointT> PointCloud;
//        static bool SetSortZ(PointT &p1, PointT &p2);
//        PointCloud::Ptr boundary;

        typedef std::pair<MapPlane*, MapPlane*> PartialManhattan;
        typedef std::tuple<MapPlane*, MapPlane*, MapPlane*> Manhattan;
        typedef std::unordered_map<PartialManhattan, KeyFrame*, PartialManhattanMapHash, PartialManhattanMapEqual> PartialManhattans;
        typedef std::unordered_map<Manhattan, KeyFrame*, ManhattanMapHash, ManhattanMapEqual> Manhattans;

        Map();

        void AddKeyFrame(KeyFrame* pKF);
        void AddMapPoint(MapPoint* pMP);
        void ComputeCrossLine(const std::vector<MapPlane*> &vpMapPlanes, double threshold, double threshold1);
        double PointDistanceFromPlane(const cv::Mat &plane, PointCloud::Ptr pointCloud);
        double PointToPlaneDistance(const cv::Mat &plane, pcl::PointXYZRGB &point);
        void AddCrossLineToMap(MapPlane*, MapPlane*,cv::Mat);
        void AddCrossPointToMap(MapPlane*, MapPlane* ,MapPlane*, cv::Mat);
        void EraseMapPoint(MapPoint* pMP);
        void EraseKeyFrame(KeyFrame* pKF);
        void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

        void InformNewBigChange();
        int GetLastBigChangeIdx();
        void AddMapLine(MapLine* pML);
        void EraseMapLine(MapLine* pML);
        void SetReferenceMapLines(const std::vector<MapLine*> &vpMLs);

        std::vector<KeyFrame*> GetAllKeyFrames();
        std::vector<MapPoint*> GetAllMapPoints();
        std::vector<MapPoint*> GetReferenceMapPoints();
        std::vector<cv::Mat> GetAllCrossLines();
        std::vector<cv::Mat> GetAllCrossPoints();

        std::vector<MapLine*> GetAllMapLines();
        std::vector<MapLine*> GetReferenceMapLines();
        long unsigned int MapLinesInMap();

        long unsigned int MapPointsInMap();
        long unsigned  KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        vector<KeyFrame*> mvpKeyFrameOrigins;
        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;
        std::mutex mMutexLineCreation;

        void AddMapPlane(MapPlane* pMP);
        void AddMapPlaneBoundary(MapPlane* pMP);
        std::vector<MapPlane*> GetAllMapPlaneBoundary();
        void EraseMapPlane(MapPlane *pMP);
        std::vector<MapPlane*> GetAllMapPlanes();
        long unsigned int MapPlanesInMap();

        void FlagMatchedPlanePoints(ORB_SLAM2::Frame &pF, const float &dTh);

        void AddManhattanObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3, KeyFrame* pKF);
        KeyFrame * GetManhattanObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3);
        Manhattans GetAllManhattanObservations();

        void AddTuplePlaneObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3, KeyFrame* pKF);
        KeyFrame * GetTuplePlaneObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3);
        TuplePlanes GetAllTuplePlaneObservations();

        void AddPairPlanesObservation(MapPlane *pMP1, MapPlane *pMP2, KeyFrame* pKF);
        KeyFrame * GetCrossLineObservation(MapPlane *pMP1, MapPlane *pMP2);
        PairPlanes GetAllPairPlaneObservation();

        void AddPartialManhattanObservation(MapPlane *pMP1, MapPlane *pMP2, KeyFrame* pKF);
        KeyFrame * GetPartialManhattanObservation(MapPlane *pMP1, MapPlane *pMP2);
        PartialManhattans GetAllPartialManhattanObservations();

        void AddBoundaryLine(Eigen::Matrix<double ,6 , 1> &boundaryLine);
        vector<Eigen::Matrix<double ,6 , 1>> GetAllPlaneIntersections();

        std::vector<SurfelElement> mvLocalSurfels;
        std::vector<SurfelElement> mvInactiveSurfels;

    protected:
        std::tuple<unsigned long,unsigned long,cv::Mat> CrossLineSet;
        vector<cv::Mat> CrossLineDraw;
        vector<std::tuple<unsigned long,unsigned long,cv::Mat>> CrossLineSets;
        std::tuple<unsigned long ,unsigned long ,unsigned long ,cv::Mat> CrossPointSet;
        vector<cv::Mat> CrossPointDraw;
        vector<std::tuple<unsigned long ,unsigned long ,unsigned long ,cv::Mat>> CrossPointSets;
        std::set<MapPoint*> mspMapPoints;

        std::set<MapLine*> mspMapLines;
        std::vector<Eigen::Matrix<double ,6 , 1>> mspBoundaryLines;

        std::set<MapPlane*> mspMapPlanes;
        std::set<MapPlane*> mspMapPlanesBoundaries;

        std::set<KeyFrame*> mspKeyFrames;

        PartialManhattans mmpPartialManhattanObservations;
        PairPlanes mmpPairPlanesObservations;

        Manhattans mmpManhattanObservations;
        TuplePlanes mmpTuplePlanesObservations;

        std::vector<MapPoint*> mvpReferenceMapPoints;
        std::vector<MapLine*> mvpReferenceMapLines;
        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;
        std::mutex mMutexMap;
    };


} //namespace ORB_SLAM

#endif // MAP_H
