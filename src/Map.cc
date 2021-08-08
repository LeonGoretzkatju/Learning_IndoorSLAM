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

#include "Map.h"

#include<mutex>

using namespace std;
using namespace cv;
using namespace cv::line_descriptor;
using namespace Eigen;

namespace ORB_SLAM2 {

    size_t PartialManhattanMapHash::operator() (const std::pair<MapPlane*, MapPlane*>& key) const {
        int id1, id2;
        if (key.first->mnId > key.second->mnId) {
            id1 = key.second->mnId;
            id2 = key.first->mnId;
        } else {
            id1 = key.first->mnId;
            id2 = key.second->mnId;
        }

        size_t hash = 0;
        hash += (71*hash + id1) % 5;
        hash += (71*hash + id2) % 5;
        return hash;
    }

    size_t PairPlaneMapHash::operator() (const std::pair<MapPlane*, MapPlane*>& key) const {
        int id1, id2;
        if (key.first->mnId > key.second->mnId) {
            id1 = key.second->mnId;
            id2 = key.first->mnId;
        } else {
            id1 = key.first->mnId;
            id2 = key.second->mnId;
        }

        size_t hash = 0;
        hash += (71*hash + id1) % 5;
        hash += (71*hash + id2) % 5;
        return hash;
    }

    bool PartialManhattanMapEqual::operator() (const std::pair<MapPlane*, MapPlane*>& a, const std::pair<MapPlane*, MapPlane*>& b) const {
        MapPlane* pMP11, *pMP12, *pMP21, *pMP22;
        if (a.first->mnId > a.second->mnId) {
            pMP11 = a.second;
            pMP12 = a.first;
        } else {
            pMP11 = a.first;
            pMP12 = a.second;
        }

        if (b.first->mnId > b.second->mnId) {
            pMP21 = b.second;
            pMP22 = b.first;
        } else {
            pMP21 = b.first;
            pMP22 = b.second;
        }

        std::pair<MapPlane*, MapPlane*> p1 = std::make_pair(pMP11, pMP12);
        std::pair<MapPlane*, MapPlane*> p2 = std::make_pair(pMP21, pMP22);

        return p1 == p2;
    }

    bool PairPlaneMapEqual::operator() (const std::pair<MapPlane*, MapPlane*>& a, const std::pair<MapPlane*, MapPlane*>& b) const {
        MapPlane* pMP11, *pMP12, *pMP21, *pMP22;
        if (a.first->mnId > a.second->mnId) {
            pMP11 = a.second;
            pMP12 = a.first;
        } else {
            pMP11 = a.first;
            pMP12 = a.second;
        }

        if (b.first->mnId > b.second->mnId) {
            pMP21 = b.second;
            pMP22 = b.first;
        } else {
            pMP21 = b.first;
            pMP22 = b.second;
        }

        std::pair<MapPlane*, MapPlane*> p1 = std::make_pair(pMP11, pMP12);
        std::pair<MapPlane*, MapPlane*> p2 = std::make_pair(pMP21, pMP22);

        return p1 == p2;
    }

    size_t TuplePlaneMapHash::operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& key) const {
        vector<int> ids;
        ids.push_back(get<0>(key)->mnId);
        ids.push_back(get<1>(key)->mnId);
        ids.push_back(get<2>(key)->mnId);
        sort(ids.begin(), ids.end());

        size_t hash = 0;
        hash += (71*hash + ids[0]) % 5;
        hash += (71*hash + ids[1]) % 5;
        hash += (71*hash + ids[2]) % 5;
        return hash;
    }

    bool TuplePlaneMapEqual::operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& a,
                                        const std::tuple<MapPlane*, MapPlane*, MapPlane*>& b) const {
        MapPlane* pMP11, *pMP12, *pMP13, *pMP21, *pMP22, *pMP23;

        pMP11 = get<0>(a);
        pMP12 = get<1>(a);
        pMP13 = get<2>(a);

        if (pMP11 > pMP12)
        {
            std::swap(pMP11, pMP12);
        }
        if (pMP12 > pMP13)
        {
            std::swap(pMP12, pMP13);
        }
        if (pMP11 > pMP12)
        {
            std::swap(pMP11, pMP12);
        }

        pMP21 = get<0>(b);
        pMP22 = get<1>(b);
        pMP23 = get<2>(b);

        if (pMP21 > pMP22)
        {
            std::swap(pMP21, pMP22);
        }
        if (pMP22 > pMP23)
        {
            std::swap(pMP22, pMP23);
        }
        if (pMP21 > pMP22)
        {
            std::swap(pMP21, pMP22);
        }

        std::tuple<MapPlane*, MapPlane*, MapPlane*> t1 = std::make_tuple(pMP11, pMP12, pMP13);
        std::tuple<MapPlane*, MapPlane*, MapPlane*> t2 = std::make_tuple(pMP21, pMP22, pMP23);

        return t1 == t2;
    }

    size_t ManhattanMapHash::operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& key) const {
        vector<int> ids;
        ids.push_back(get<0>(key)->mnId);
        ids.push_back(get<1>(key)->mnId);
        ids.push_back(get<2>(key)->mnId);
        sort(ids.begin(), ids.end());

        size_t hash = 0;
        hash += (71*hash + ids[0]) % 5;
        hash += (71*hash + ids[1]) % 5;
        hash += (71*hash + ids[2]) % 5;
        return hash;
    }

    bool ManhattanMapEqual::operator() (const std::tuple<MapPlane*, MapPlane*, MapPlane*>& a,
                                        const std::tuple<MapPlane*, MapPlane*, MapPlane*>& b) const {
        MapPlane* pMP11, *pMP12, *pMP13, *pMP21, *pMP22, *pMP23;

        pMP11 = get<0>(a);
        pMP12 = get<1>(a);
        pMP13 = get<2>(a);

        if (pMP11 > pMP12)
        {
            std::swap(pMP11, pMP12);
        }
        if (pMP12 > pMP13)
        {
            std::swap(pMP12, pMP13);
        }
        if (pMP11 > pMP12)
        {
            std::swap(pMP11, pMP12);
        }

        pMP21 = get<0>(b);
        pMP22 = get<1>(b);
        pMP23 = get<2>(b);

        if (pMP21 > pMP22)
        {
            std::swap(pMP21, pMP22);
        }
        if (pMP22 > pMP23)
        {
            std::swap(pMP22, pMP23);
        }
        if (pMP21 > pMP22)
        {
            std::swap(pMP21, pMP22);
        }

        std::tuple<MapPlane*, MapPlane*, MapPlane*> t1 = std::make_tuple(pMP11, pMP12, pMP13);
        std::tuple<MapPlane*, MapPlane*, MapPlane*> t2 = std::make_tuple(pMP21, pMP22, pMP23);

        return t1 == t2;
    }

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange() {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear() {
        for (auto mspMapPoint : mspMapPoints)
            delete mspMapPoint;
        for (auto mspMapLine : mspMapLines)
            delete mspMapLine;
        for (auto mspMapPlane : mspMapPlanes)
            delete mspMapPlane;

        for (auto mspKeyFrame : mspKeyFrames)
            delete mspKeyFrame;

        mspMapPlanes.clear();
        mspMapPoints.clear();
        mspKeyFrames.clear();
        mspMapLines.clear();
        mspBoundaryLines.clear();
        mspDirectionVector.clear();
        BoundaryPoints.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpReferenceMapLines.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::AddMapLine(MapLine *pML) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapLines.insert(pML);
    }

    void Map::AddBoundaryLine(Eigen::Matrix<double ,6 , 1> &boundaryLine) {
        unique_lock<mutex> lock(mMutexMap);
        mspBoundaryLines.emplace_back(boundaryLine);
    }

    void Map::AddDirectionVector(Eigen::Matrix<double, 3, 1> &DirectionVector) {
        unique_lock<mutex> lock(mMutexMap);
        mspDirectionVector.emplace_back(DirectionVector);
    }

//    void JudgeSimilarityDirectionVector(Eigen::Matrix<double, 3, 1> &DirectionVector, Eigen::Matrix<double ,6 , 1> &boundaryLine) {
////        unique_lock<mutex> lock(mMutexMap);
//        cout << "stop at there" << endl;
//        cout <<"mspDirectionvector size" << "     " << mspDirectionVector.size() << endl;
//        for (int i = 0; i < mspBoundaryLines.size(); ++i) {
//            float angle = DirectionVector[0]*mspDirectionVector[i][0] + DirectionVector[1]*mspDirectionVector[i][1] + DirectionVector[2]*mspDirectionVector[i][2];
////            cout <<"angle of direction vector" << "     " <<angle <<endl;
//            if (abs(angle) > 0.8)
//            {
//                continue;
//            }
//            else
//            {
//                mspDirectionVector.emplace_back(DirectionVector);
//                mspBoundaryLines.emplace_back(boundaryLine);
////                cout << "finish emplace back" << endl;
//            }
//        }
//    }

    void Map::EraseMapLine(MapLine *pML) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapLines.erase(pML);
    }

    /**
     * @brief 设置参考MapLines，将用于DrawMapLines函数画图
     * @param vpMLs Local MapLines
     */
    void Map::SetReferenceMapLines(const std::vector<MapLine *> &vpMLs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapLines = vpMLs;
    }

    vector<MapLine *> Map::GetAllMapLines() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapLine *>(mspMapLines.begin(), mspMapLines.end());
    }

    vector<Eigen::Matrix<double ,6 , 1>> Map::GetAllPlaneIntersections() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<Eigen::Matrix<double ,6 , 1>>(mspBoundaryLines.begin(), mspBoundaryLines.end());
    }

    vector<MapLine *> Map::GetReferenceMapLines() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapLines;
    }

    long unsigned int Map::MapLinesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapLines.size();
    }

    void Map::AddMapPlane(MapPlane *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        pMP->mvPlanePoints.get()->points;
        pMP->mvNoPlanePoints.get()->points;
        mspMapPlanes.insert(pMP);
    }

    void Map::AddBoundaryPoints(pcl::PointXYZRGB &p) {
        unique_lock<mutex> lock(mMutexMap);
        BoundaryPoints.emplace_back(p);
    }

    void Map::AddInlierLines(pcl::PointXYZRGB &p) {
        unique_lock<mutex> lock(mMutexMap);
        InlierLines.emplace_back(p);
    }

    bool SetSortZ(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2) {
        if (p1.z != p2.z)
            return p1.z < p2.z;
        else
            if(p1.x != p2.x)
                return p1.x < p2.x;
            else
                if(p1.y != p2.y)
                    return p1.y < p2.y;
    }

    double Map::PointDistanceFromPlane(const cv::Mat &plane, PointCloud::Ptr pointCloud, double minSize) {
        double sum = 0;
        for (int i = 0; i < minSize; ++i){
//            cout <<"p x" << p.x << endl;
            double dis = abs(plane.at<float>(0, 0) * pointCloud->points[i].x +
                             plane.at<float>(1, 0) * pointCloud->points[i].y +
                             plane.at<float>(2, 0) * pointCloud->points[i].z +
                             plane.at<float>(3, 0));
//            cout << "compute 1 iteration" << endl;
            sum += dis;
        }
        return sum/minSize;
    }

    double Map::PointToPlaneDistance(const cv::Mat &plane, pcl::PointXYZRGB &point) {
        double dis = abs(plane.at<float>(0, 0) * point.x +
                         plane.at<float>(1, 0) * point.y +
                         plane.at<float>(2, 0) * point.z +
                         plane.at<float>(3, 0));
        return dis;
    }

    void Map::ComputeCrossLine(const std::vector<MapPlane*> &vpMapPlanes, double threshold, double threshold1) {
//        unique_lock<mutex> lock(mMutexMap); threshold 2.0 2.5 .etc
//        PointCloud::Ptr boundary (new PointCloud());
        for(int i = 0; i < vpMapPlanes.size(); i++) {
            for (int j = i+1; j < vpMapPlanes.size(); j++) {
                PointCloud::Ptr boundary (new PointCloud());
                int minSize = min(vpMapPlanes[i]->mvPlanePoints->points.size(),vpMapPlanes[j]->mvPlanePoints->points.size());
                cv::Mat p1 = vpMapPlanes[i]->GetWorldPos();
                cv::Mat p2 = vpMapPlanes[j]->GetWorldPos();
                float angle = p1.at<float>(0,0)*p2.at<float>(0,0) + p1.at<float>(1,0)*p2.at<float>(1,0)+
                              p1.at<float>(2,0)*p1.at<float>(2,0);
//                cout << "plane parameter p1" << "            " <<p1 <<endl;
//                cout << "v1pmapplane size" <<vpMapPlanes[j]->mvPlanePoints->points.size() << endl;
                double dis = PointDistanceFromPlane(p1,vpMapPlanes[j]->mvPlanePoints, minSize);
                cout << "total distance" << "                    " <<dis <<endl;
                cout << "total angle" << "           " <<angle <<endl;
//                cout <<"success compute the distance from plane to plane" <<endl;
                if (angle < 0.28716 || dis < threshold) {
                    threshold = dis;
                    if (threshold < 0.5)
                        threshold = 0.5;
                    for (auto p : vpMapPlanes[j]->mvPlanePoints->points) {
//                        cout << "point to plane distance" << "       " << PointToPlaneDistance(p1, p) << endl;
                        if (PointToPlaneDistance(p1, p) < threshold1)
                        {
                            threshold1 = PointToPlaneDistance(p1, p);
                            if (threshold1 < 0.005)
                                threshold1 = 0.005;
//                            cout << "point to plane distance" << "       " << PointToPlaneDistance(p1, p) << endl;
                            boundary->points.emplace_back(p);
                            AddBoundaryPoints(p);
                        }
                    }
                    for (auto pp : vpMapPlanes[i]->mvPlanePoints->points) {
                        if (PointToPlaneDistance(vpMapPlanes[j]->GetWorldPos(), pp) < threshold1)
                        {
                            threshold1 = PointToPlaneDistance(vpMapPlanes[j]->GetWorldPos(), pp);
                            if (threshold1 < 0.01)
                                threshold1 = 0.01;
                            boundary->points.emplace_back(pp);
                            AddBoundaryPoints(pp);
                        }
                    }
                    cout << "boundary points" << "               " << boundary->points.size() <<endl;
                    if (boundary->points.size() >= 7)
                    {
                        std::sort(boundary->points.begin(), boundary->points.end(), SetSortZ);
                        cout << "finish sort" << endl;
                        pcl::PointXYZRGB point1 = boundary->points[0];
                        pcl::PointXYZRGB point2 = boundary->points[boundary->points.size() - 1];
                        pcl::PointXYZRGB ProjectLeft1, ProjectLeft2, ProjectRight1, ProjectRight2;
                        pcl::PointXYZRGB UpCrossPoint, DownCrossPoint;
                        float tLeft1 = (vpMapPlanes[i]->GetWorldPos()).at<float>(0, 0) * point1.x +
                                       (vpMapPlanes[i]->GetWorldPos()).at<float>(1, 0) * point1.y +
                                       (vpMapPlanes[i]->GetWorldPos()).at<float>(2, 0) * point1.z +
                                       (vpMapPlanes[i]->GetWorldPos()).at<float>(3, 0);
                        float tLeft2 = (vpMapPlanes[i]->GetWorldPos()).at<float>(0, 0) * point2.x +
                                       (vpMapPlanes[i]->GetWorldPos()).at<float>(1, 0) * point2.y +
                                       (vpMapPlanes[i]->GetWorldPos()).at<float>(2, 0) * point2.z +
                                       (vpMapPlanes[i]->GetWorldPos()).at<float>(3, 0);
                        float tRight1 = (vpMapPlanes[j]->GetWorldPos()).at<float>(0, 0) * point1.x +
                                        (vpMapPlanes[j]->GetWorldPos()).at<float>(1, 0) * point1.y +
                                        (vpMapPlanes[j]->GetWorldPos()).at<float>(2, 0) * point1.z +
                                        (vpMapPlanes[j]->GetWorldPos()).at<float>(3, 0);
                        float tRight2 = (vpMapPlanes[j]->GetWorldPos()).at<float>(0, 0) * point2.x +
                                        (vpMapPlanes[j]->GetWorldPos()).at<float>(1, 0) * point2.y +
                                        (vpMapPlanes[j]->GetWorldPos()).at<float>(2, 0) * point2.z +
                                        (vpMapPlanes[j]->GetWorldPos()).at<float>(3, 0);
                        ProjectLeft1.x = point1.x - (vpMapPlanes[i]->GetWorldPos()).at<float>(0, 0) * tLeft1;
                        ProjectLeft1.y = point1.y - (vpMapPlanes[i]->GetWorldPos()).at<float>(1, 0) * tLeft1;
                        ProjectLeft1.z = point1.z - (vpMapPlanes[i]->GetWorldPos()).at<float>(2, 0) * tLeft1;
                        ProjectRight1.x = point1.x - (vpMapPlanes[j]->GetWorldPos()).at<float>(0, 0) * tRight1;
                        ProjectRight1.y = point1.y - (vpMapPlanes[j]->GetWorldPos()).at<float>(1, 0) * tRight1;
                        ProjectRight1.z = point1.z - (vpMapPlanes[j]->GetWorldPos()).at<float>(2, 0) * tRight1;
                        ProjectLeft2.x = point2.x - (vpMapPlanes[i]->GetWorldPos()).at<float>(0, 0) * tLeft2;
                        ProjectLeft2.y = point2.y - (vpMapPlanes[i]->GetWorldPos()).at<float>(1, 0) * tLeft2;
                        ProjectLeft2.z = point2.z - (vpMapPlanes[i]->GetWorldPos()).at<float>(2, 0) * tLeft2;
                        ProjectRight2.x = point2.x - (vpMapPlanes[j]->GetWorldPos()).at<float>(0, 0) * tRight2;
                        ProjectRight2.y = point2.y - (vpMapPlanes[j]->GetWorldPos()).at<float>(1, 0) * tRight2;
                        ProjectRight2.z = point2.z - (vpMapPlanes[j]->GetWorldPos()).at<float>(2, 0) * tRight2;
                        cv::Mat pN1 = vpMapPlanes[i]->GetWorldPos();
                        cv::Mat pN2 = vpMapPlanes[j]->GetWorldPos();
                        pN1 = (cv::Mat_<float>(3, 1) << pN1.at<float>(0), pN1.at<float>(1), pN1.at<float>(2));
                        pN2 = (cv::Mat_<float>(3, 1) << pN2.at<float>(0), pN2.at<float>(1), pN2.at<float>(2));
                        cv::Mat CrossLine = pN1.cross(pN2);
                        cv::Mat A, bup, bdown;
                        A = cv::Mat::eye(cv::Size(3, 3), CV_32F);

                        A.at<float>(0, 0) = pN1.at<float>(0);
                        A.at<float>(1, 0) = pN2.at<float>(0);
                        A.at<float>(2, 0) = CrossLine.at<float>(0);
                        A.at<float>(0, 1) = pN1.at<float>(1);
                        A.at<float>(1, 1) = pN2.at<float>(1);
                        A.at<float>(2, 1) = CrossLine.at<float>(1);
                        A.at<float>(0, 2) = pN1.at<float>(2);
                        A.at<float>(1, 2) = pN2.at<float>(2);
                        A.at<float>(2, 2) = CrossLine.at<float>(2);
                        float result = CrossLine.at<float>(0) * (ProjectLeft1.x - ProjectRight1.x) + CrossLine.at<float>(1) * (ProjectLeft1.y - ProjectRight1.y) +
                                       CrossLine.at<float>(2) * (ProjectLeft1.z - ProjectRight1.z);
                        cout << "final result" << "                   " << result << endl;
//                        A = A.t() * A;
//                        float b1 = pN1.at<float>(0) * ProjectLeft1.x + pN1.at<float>(1) * ProjectLeft1.y +
//                                   pN1.at<float>(2) * ProjectLeft1.z;
                        float b1 = -pN1.at<float>(3);
//                        float b2 = pN2.at<float>(0) * ProjectRight1.x + pN2.at<float>(1) * ProjectRight1.y +
//                                   pN2.at<float>(2) * ProjectRight1.z;
                        float b2 = -pN2.at<float>(3);
                        float b3 = CrossLine.at<float>(0) * ProjectLeft1.x + CrossLine.at<float>(1) * ProjectLeft1.y +
                                   CrossLine.at<float>(2) * ProjectLeft1.z;
                        bup = (cv::Mat_<float>(3, 1) << b1, b2, b3);
//                        bup = A.t() * bup;
                        cv::Mat CrossPointSet = A.inv() * bup;
                        UpCrossPoint.x = CrossPointSet.at<float>(0);
                        UpCrossPoint.y = CrossPointSet.at<float>(1);
                        UpCrossPoint.z = CrossPointSet.at<float>(2);
//                        float b11 = pN1.at<float>(0) * ProjectLeft2.x + pN1.at<float>(1) * ProjectLeft2.y +
//                                    pN1.at<float>(2) * ProjectLeft2.z;
//                        float b22 = pN2.at<float>(0) * ProjectRight2.x + pN2.at<float>(1) * ProjectRight2.y +
//                                    pN2.at<float>(2) * ProjectRight2.z;
                        float b11 = -pN1.at<float>(3);
                        float b22 = -pN2.at<float>(3);
                        float b33 = CrossLine.at<float>(0) * ProjectRight2.x + CrossLine.at<float>(1) * ProjectRight2.y +
                                    CrossLine.at<float>(2) * ProjectRight2.z;
                        bdown = (cv::Mat_<float>(3, 1) << b11, b22, b33);
//                        bdown = A.t() * bdown;
                        cv::Mat DownCrossPointSet = A.inv() * bdown;
                        DownCrossPoint.x = DownCrossPointSet.at<float>(0);
                        DownCrossPoint.y = DownCrossPointSet.at<float>(1);
                        DownCrossPoint.z = DownCrossPointSet.at<float>(2);
                        Eigen::Matrix<double, 6, 1> boundaryLine;
                        Eigen::Matrix<double, 3, 1> DirectionVector;
                        boundaryLine
                                << ProjectLeft1.x, ProjectLeft1.y, ProjectLeft1.z, ProjectLeft2.x, ProjectLeft2.y, ProjectLeft2.z;
                        double base = sqrt(pow((ProjectLeft2.x - ProjectLeft1.x), 2) + pow((ProjectLeft2.y - ProjectLeft1.y), 2) + pow((ProjectLeft2.z - ProjectLeft1.z), 2));
                        DirectionVector << (ProjectLeft2.x - ProjectLeft1.x)/base, (ProjectLeft2.y - ProjectLeft1.y)/base, (ProjectLeft2.z - ProjectLeft1.z)/base;
//                        AddBoundaryLine(boundaryLine);
//                            mspBoundaryLines.emplace_back(boundaryLine);
//                            mspDirectionVector.emplace_back(DirectionVector);
//                        AddDirectionVector(DirectionVector);
//                        cout << "mspBoundaryLines" << "                      " << mspBoundaryLines.size() << endl;
//                        cout << "mspDirectionVectorSize" << "             " << mspDirectionVector.size() <<endl;
                        if (TupleDirectVector.empty())
                        {
                            auto TupleDirect = std::make_tuple(vpMapPlanes[i]->mnId, vpMapPlanes[j]->mnId, DirectionVector);
                            TupleDirectVector.emplace_back(TupleDirect);
                            AddBoundaryLine(boundaryLine);
                        }
                        else if (TupleDirectVector.size() == 1)
                        {
                            auto id1 = std::get<0>(TupleDirectVector[0]);
                            auto id2 = std::get<1>(TupleDirectVector[1]);
                            if (vpMapPlanes[i]->mnId != id1 || vpMapPlanes[j]->mnId != id2)
                            {
                                auto TupleDirect = std::make_tuple(vpMapPlanes[i]->mnId, vpMapPlanes[j]->mnId, DirectionVector);
                                TupleDirectVector.emplace_back(TupleDirect);
                                AddBoundaryLine(boundaryLine);
                            }
                        }
                        else
                        {
                            bool flag = false;
                            bool sum = false;
                            for (int k = 0; k < TupleDirectVector.size(); ++k) {
                                auto idOne = std::get<0>(TupleDirectVector[k]);
                                auto idTwo = std::get<1>(TupleDirectVector[k]);
                                if (vpMapPlanes[i]->mnId == idOne && vpMapPlanes[j]->mnId == idTwo)
                                {
                                    float angle = DirectionVector[0]*std::get<2>(TupleDirectVector[k])(0) +
                                            DirectionVector[1]*std::get<2>(TupleDirectVector[k])(1) +
                                            DirectionVector[2]*std::get<2>(TupleDirectVector[k])(2);
                                    if (angle > 0)
                                        angle = acos(angle);
                                    else
                                        angle = CV_PI - acos(angle);
                                    cout <<"angle of direction vector" << "     " <<angle <<endl;
                                    if (angle >= 4*CV_PI/9 && angle <= 0.5*CV_PI)
                                    {
                                        auto Match = std::make_tuple(vpMapPlanes[i]->mnId, vpMapPlanes[j]->mnId, DirectionVector);
                                        TupleDirectVector.emplace_back(Match);
                                    }
                                    flag = true;
                                }
                                else
                                {
                                    flag = false;
                                }
                                sum += flag;
                            }
                            if (!flag)
                            {
                                AddBoundaryLine(boundaryLine);
                            }

                        }
//                        float mCurrentLineSize;
//                        if (mspBoundaryLines.size() == 0)
//                        {
//                            AddBoundaryLine(boundaryLine);
////                            mspBoundaryLines.emplace_back(boundaryLine);
////                            mspDirectionVector.emplace_back(DirectionVector);
//                            AddDirectionVector(DirectionVector);
//                            cout << "mspBoundaryLines" << "                      " << mspBoundaryLines.size() << endl;
//                            cout << "mspDirectionVectorSize" << "             " << mspDirectionVector.size() <<endl;
//                            mCurrentLineSize = 1;
//                        }
//                        else
//                        {
//                            float angle = DirectionVector[0]*mspDirectionVector[mCurrentLineSize-1](0) +
//                                          DirectionVector[1]*mspDirectionVector[mCurrentLineSize-1](1) +
//                                          DirectionVector[2]*mspDirectionVector[mCurrentLineSize-1](2);
//                            if (angle > 0)
//                                angle = acos(angle);
//                            else
//                                angle = CV_PI - acos(angle);
//                            cout <<"angle of direction vector" << "     " <<angle <<endl;
//                            if (angle >= 4*CV_PI/9 && angle <= 0.5*CV_PI)
//                            {
//
//                            }
//                            bool flag = false;
//                            std::vector<bool> VectorBool;
//                            for (int i = 0; i < mspBoundaryLines.size(); ++i) {
//                                cout << "mspDirectionVector" << "             " << mspDirectionVector[i] << endl;
//                                float angle = DirectionVector[0]*mspDirectionVector[i](0) + DirectionVector[1]*mspDirectionVector[i](1) + DirectionVector[2]*mspDirectionVector[i](2);
//                                if (angle > 0)
//                                    angle = acos(angle);
//                                else
//                                    angle = CV_PI - acos(angle);
//                                cout <<"angle of direction vector" << "     " <<angle <<endl;
//                                if (angle >= 0.0 && angle <= 4*CV_PI/9)
//                                {
//                                    flag = false;
//                                }
//                                else
//                                {
//                                    flag = true;
//                                }
//                                VectorBool.emplace_back(flag);
//                            }
//                            bool flag1 = true;
//                            for (int k = 0; k < VectorBool.size(); ++k) {
//                                flag1 = flag1 * VectorBool[k];
//                            }
//                            if (flag1)
//                            {
//                                AddBoundaryLine(boundaryLine);
//                                AddDirectionVector(DirectionVector);
//                                cout << "mspBoundaryLines" << "                      " << mspBoundaryLines.size() << endl;
//                                cout << "mspDirectionVectorSize" << "             " << mspDirectionVector.size() <<endl;
//                            }
//                        }
                        cout << "finish boundary line" << endl;
                    }
                }
            }
        }
            cout << "finish this step" << endl;
    }

    void Map::AddMapPlaneBoundary(MapPlane* pMP){
        unique_lock<mutex> lock(mMutexMap);
        pMP->cloud_boundary.get()->points;
        mspMapPlanesBoundaries.insert(pMP);
    }

    void Map::EraseMapPlane(MapPlane *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPlanes.erase(pMP);
    }

    vector<MapPlane *> Map::GetAllMapPlanes() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPlane *>(mspMapPlanes.begin(), mspMapPlanes.end());
    }

    vector<MapPlane *> Map::GetAllMapPlaneBoundary() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPlane *>(mspMapPlanesBoundaries.begin(), mspMapPlanesBoundaries.end());
    }

    vector<pcl::PointXYZRGB> Map::GetAllInlierLines() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<pcl::PointXYZRGB>(InlierLines.begin(), InlierLines.end());
    }


    vector<cv::Mat> Map::GetAllCrossLines() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<cv::Mat>(CrossLineDraw.begin(),CrossLineDraw.end());
    }

    vector<pcl::PointXYZRGB> Map::GetAllBoundaryPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<pcl::PointXYZRGB>(BoundaryPoints.begin(), BoundaryPoints.end());
    }

    vector<cv::Mat> Map::GetAllCrossPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<cv::Mat>(CrossPointDraw.begin(),CrossPointDraw.end());
    }

    long unsigned int Map::MapPlanesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPlanes.size();
    }

    void Map::FlagMatchedPlanePoints(ORB_SLAM2::Frame &pF, const float &dTh) {
//match plane points based on the distance between the point and the plane
        unique_lock<mutex> lock(mMutexMap);
        int nMatches = 0;

        for (int i = 0; i < pF.mnPlaneNum; ++i) {

            cv::Mat pM = pF.ComputePlaneWorldCoeff(i);

            if (pF.mvpMapPlanes[i]) {
                for (auto mapPoint : mspMapPoints) {
                    cv::Mat pW = mapPoint->GetWorldPos();

                    double dis = abs(pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                                     pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                                     pM.at<float>(2, 0) * pW.at<float>(2, 0) +
                                     pM.at<float>(3, 0));

                    if (dis < 0.5) {
                        mapPoint->SetAssociatedWithPlaneFlag(true);
                        nMatches++;
                    }
                }
            }
        }

//        cout << "Point matches: " << nMatches << endl;
    }

    void Map::AddTuplePlaneObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3, KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexMap);

        TuplePlane planes = std::make_tuple(pMP1, pMP2, pMP3);
        if (mmpTuplePlanesObservations.count(planes) != 0)
            return;
//        cout << "Insert Manhattan3 pMP1: " << pMP1->mnId << endl;
//        cout << "Insert Manhattan3 pMP2: " << pMP2->mnId << endl;
//        cout << "Insert Manhattan3 pMP3: " << pMP3->mnId << endl;
        pKF->SetNotErase();
        mmpTuplePlanesObservations[planes] = pKF;
    }

    KeyFrame* Map::GetTuplePlaneObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3) {
        unique_lock<mutex> lock(mMutexMap);
        TuplePlane planes = std::make_tuple(pMP1, pMP2, pMP3);
        if (mmpTuplePlanesObservations.count(planes)) {
            return mmpTuplePlanesObservations[planes];
        } else {
            return static_cast<KeyFrame*>(nullptr);
        }
    }

    void Map::AddManhattanObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3, KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexMap);

        Manhattan manhattan = std::make_tuple(pMP1, pMP2, pMP3);
        if (mmpManhattanObservations.count(manhattan) != 0)
            return;
//        cout << "Insert Manhattan3 pMP1: " << pMP1->mnId << endl;
//        cout << "Insert Manhattan3 pMP2: " << pMP2->mnId << endl;
//        cout << "Insert Manhattan3 pMP3: " << pMP3->mnId << endl;
        pKF->SetNotErase();
        mmpManhattanObservations[manhattan] = pKF;
    }

    KeyFrame* Map::GetManhattanObservation(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3) {
        unique_lock<mutex> lock(mMutexMap);
        Manhattan manhattan = std::make_tuple(pMP1, pMP2, pMP3);
        if (mmpManhattanObservations.count(manhattan)) {
            return mmpManhattanObservations[manhattan];
        } else {
            return static_cast<KeyFrame*>(nullptr);
        }
    }

    void Map::AddCrossLineToMap(MapPlane *pMP1, MapPlane *pMP2, cv::Mat CrossLine) {
        unique_lock<mutex> lock(mMutexMap);
        CrossLineSet = std::make_tuple(pMP1->mnId,pMP2->mnId,CrossLine);
        CrossLineDraw.emplace_back(CrossLine);
        CrossLineSets.emplace_back(CrossLineSet);
    }

    void Map::AddCrossPointToMap(MapPlane *pMP1, MapPlane *pMP2, MapPlane *pMP3, cv::Mat CrossPoint) {
        unique_lock<mutex> lock(mMutexMap);
        CrossPointSet = std::make_tuple(pMP1->mnId,pMP2->mnId,pMP3->mnId,CrossPoint);
        CrossPointDraw.emplace_back(CrossPoint);
        CrossPointSets.emplace_back(CrossPointSet);
    }

    void Map::AddPairPlanesObservation(MapPlane *pMP1, MapPlane *pMP2, KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexMap);

        PairPlane plane = std::make_pair(pMP1, pMP2);
//        cout << "Insert Manhattan2 pMP1: " << pMP1->mnId << endl;
//        cout << "Insert Manhattan2 pMP2: " << pMP2->mnId << endl;
        if (mmpPairPlanesObservations.count(plane) != 0)
            return;
        pKF->SetNotErase();
        mmpPairPlanesObservations[plane] = pKF;
    }

    KeyFrame* Map::GetCrossLineObservation(MapPlane *pMP1, MapPlane *pMP2) {
        unique_lock<mutex> lock(mMutexMap);
        PairPlane plane = std::make_pair(pMP1, pMP2);
        if (mmpPairPlanesObservations.count(plane)) {
            return mmpPairPlanesObservations[plane];
        } else {
            return static_cast<KeyFrame*>(nullptr);
        }
    }

    Map::PairPlanes Map::GetAllPairPlaneObservation() {
        return mmpPairPlanesObservations;
    }

    Map::Manhattans Map::GetAllManhattanObservations() {
        return mmpManhattanObservations;
    }

    Map::TuplePlanes Map::GetAllTuplePlaneObservations() {
        return mmpTuplePlanesObservations;
    }

    void Map::AddPartialManhattanObservation(MapPlane *pMP1, MapPlane *pMP2, KeyFrame* pKF) {
        unique_lock<mutex> lock(mMutexMap);

        PartialManhattan manhattan = std::make_pair(pMP1, pMP2);
//        cout << "Insert Manhattan2 pMP1: " << pMP1->mnId << endl;
//        cout << "Insert Manhattan2 pMP2: " << pMP2->mnId << endl;
        if (mmpPartialManhattanObservations.count(manhattan) != 0)
            return;
        pKF->SetNotErase();
        mmpPartialManhattanObservations[manhattan] = pKF;
    }

    KeyFrame* Map::GetPartialManhattanObservation(MapPlane *pMP1, MapPlane *pMP2) {
        unique_lock<mutex> lock(mMutexMap);
        PartialManhattan manhattan = std::make_pair(pMP1, pMP2);
        if (mmpPartialManhattanObservations.count(manhattan)) {
            return mmpPartialManhattanObservations[manhattan];
        } else {
            return static_cast<KeyFrame*>(nullptr);
        }
    }

    Map::PartialManhattans Map::GetAllPartialManhattanObservations() {
        return mmpPartialManhattanObservations;
    }

} //namespace ORB_SLAM
