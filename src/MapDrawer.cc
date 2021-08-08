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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

using namespace std;
using namespace cv;
using namespace cv::line_descriptor;
using namespace Eigen;

namespace ORB_SLAM2
{


    MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
        mLineWidth = fSettings["Viewer.LineWidth"];
    }

    void MapDrawer::DrawMapPoints()
    {
        const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);     //黑色

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
//        cout << "point\t" << i << ": " << pos.at<float>(0) << ", " << pos.at<float>(1) << ", " << pos.at<float>(2) << endl;
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);     //红色

        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }

        glEnd();
    }

    void MapDrawer::DrawBoundaryPoints() {
        const vector<pcl::PointXYZRGB> &vpMPs = mpMap->GetAllBoundaryPoints();
        if(vpMPs.empty())
            return;

        glPointSize(7);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0);     //黑色

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
//            if(vpMPs[i]->isBad())
//                continue;
            pcl::PointXYZRGB pos = vpMPs[i];
//        cout << "point\t" << i << ": " << pos.at<float>(0) << ", " << pos.at<float>(1) << ", " << pos.at<float>(2) << endl;
            glVertex3f(pos.x,pos.y,pos.z);
        }
        glEnd();
    }

    void MapDrawer::DrawInlierLines() {
        const vector<pcl::PointXYZRGB> &vpMPs = mpMap->GetAllInlierLines();
        if(vpMPs.empty())
            return;

        glPointSize(7);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,1.0);     //黑色

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
//            if(vpMPs[i]->isBad())
//                continue;
            pcl::PointXYZRGB pos = vpMPs[i];
//        cout << "point\t" << i << ": " << pos.at<float>(0) << ", " << pos.at<float>(1) << ", " << pos.at<float>(2) << endl;
            glVertex3f(pos.x,pos.y,pos.z);
        }
        glEnd();
    }

    void MapDrawer::DrawMapLines()
    {
        const vector<MapLine*> &vpMLs = mpMap->GetAllMapLines();
        const vector<MapLine*> &vpRefMLs = mpMap->GetReferenceMapLines();

        set<MapLine*> spRefMLs(vpRefMLs.begin(), vpRefMLs.end());//set中不包含任何的重复元素

        if(vpMLs.empty())
            return;

        glLineWidth(mLineWidth);
        glBegin ( GL_LINES );
//    glColor3f(0.4, 0.35, 0.8);  //紫色
        glColor3f(0.0,0.0,0.0);    //黑色

//    cout << "vpMLs.size() = " << vpMLs.size() << endl;
        for(size_t i=0, iend=vpMLs.size(); i<iend; i++)
        {
            if(vpMLs[i]->isBad() || spRefMLs.count(vpMLs[i]))//.count() return true if the value can be found in the set.
                continue;
            Vector6d pos = vpMLs[i]->GetWorldPos();
//        cout << "line = " << pos.head(3).transpose() << "\n" << pos.tail(3).transpose() << endl;

            glVertex3f(pos(0), pos(1), pos(2));
            glVertex3f(pos(3), pos(4), pos(5));

        }
        glEnd();

        glLineWidth(mLineWidth);
        glBegin ( GL_LINES );
        glColor3f(0.0,0.0,0.0); //红色
//    cout << "spRefMLs.size() = " << spRefMLs.size() << endl;

        for(set<MapLine*>::iterator sit=spRefMLs.begin(), send=spRefMLs.end(); sit!=send; sit++)
        {
//        cout << "(*sit)->isBad() = " << (*sit)->isBad() << endl;
            if((*sit)->isBad())
                continue;
            Vector6d pos = (*sit)->GetWorldPos();
//        cout << "pos = " << pos.head(3).transpose() << "\n" << pos.tail(3).transpose() << endl;
            glVertex3f(pos(0), pos(1), pos(2));
            glVertex3f(pos(3), pos(4), pos(5));
        }
        glEnd();
    }

    void MapDrawer::DrawPlaneIntersections() {
        const vector<Eigen::Matrix<double, 6, 1>> &vpMLs = mpMap->GetAllPlaneIntersections();

        if(vpMLs.empty())
            return;

        glLineWidth(5);
        glBegin ( GL_LINES );
        glColor3f(0.0,1.0,0.0);    //黑色
        for(size_t i=0, iend=vpMLs.size(); i<iend; i++)
        {

            Vector6d pos = vpMLs[i];
            glVertex3f(pos(0), pos(1), pos(2));
            glVertex3f(pos(3), pos(4), pos(5));

        }
        glEnd();
    }

    void MapDrawer::DrawMapPlanes() {
        const vector<MapPlane*> &vpMPs = mpMap->GetAllMapPlanes();
        if(vpMPs.empty())
            return;
        glPointSize(mPointSize*2);
        glBegin(GL_POINTS);

        for(auto pMP : vpMPs){
            float ir = pMP->mRed;
            float ig = pMP->mGreen;
            float ib = pMP->mBlue;
            float norm = sqrt(ir*ir + ig*ig + ib*ib);
            glColor3f(ir/norm, ig/norm, ib/norm);
            for(auto& p : pMP->mvPlanePoints.get()->points){
                glVertex3f(p.x,p.y,p.z);
            }
        }
        glEnd();
    }

    void MapDrawer::DrawNoPlaneArea() {
        const vector<MapPlane*> &vpMPs = mpMap->GetAllMapPlanes();
        if(vpMPs.empty())
            return;
        glPointSize(mPointSize*2);
        glBegin(GL_POINTS);

        for(auto pMP : vpMPs){
            float ir = 0.0;
            float ig = 0.0;
            float ib = 0.0;
            float norm = sqrt(ir*ir + ig*ig + ib*ib);
            glColor3f(ir/norm, ig/norm, ib/norm);
            for(auto& p : pMP->mvNoPlanePoints.get()->points){
                glVertex3f(p.x,p.y,p.z);
            }
        }
        glEnd();
    }

    void MapDrawer::DrawMapPlaneBoundaries() {
        const vector<MapPlane*> &vpMPs = mpMap->GetAllMapPlanes();
        if(vpMPs.empty())
            return;
        glPointSize(mPointSize*2);
        glBegin(GL_POINTS);

        for(auto pMP : vpMPs){
            float ir = 255.0;
            float ig = 0.0;
            float ib = 0.0;
            float norm = sqrt(ir*ir + ig*ig + ib*ib);
            glColor3f(ir/norm, ig/norm, ib/norm);
            if (pMP->cloud_boundary.get()->points.size() > 0)
            {
                for(auto& p : pMP->cloud_boundary.get()->points){
                    glVertex3f(p.x,p.y,p.z);
                }
            }
            else
            {
                continue;
            }
        }
        glEnd();
    }

    void MapDrawer::DrawCrossLine() {
        const vector<cv::Mat> &vpMLs = mpMap->GetAllCrossLines();
        if(vpMLs.empty())
            return;

        glLineWidth(mLineWidth);
        glBegin ( GL_LINES );
//    glColor3f(0.4, 0.35, 0.8);  //紫色
        glColor3f(0.0,0.0,0.0);    //黑色

//    cout << "vpMLs.size() = " << vpMLs.size() << endl;
        for(size_t i=0, iend=vpMLs.size(); i<iend; i++)
        {
            glVertex3f(vpMLs[i].at<float>(0), vpMLs[i].at<float>(1), vpMLs[i].at<float>(2));
        }
        glEnd();

        glLineWidth(mLineWidth);
        glBegin ( GL_LINES );
        glColor3f(0.0,0.0,0.0); //红色
        glEnd();
    }

    void MapDrawer::DrawCrossPoint() {
        const vector<cv::Mat> &vpMPs = mpMap->GetAllCrossPoints();
        if(vpMPs.empty())
            return;
        glPointSize(mPointSize*5);
        glBegin(GL_POINTS);

        for(auto pMP : vpMPs){
            float ir = 255.0;
            float ig = 255.0;
            float ib = 0.0;
            float norm = sqrt(ir*ir + ig*ig + ib*ib);
            glColor3f(ir/norm, ig/norm, ib/norm);
            glVertex3f(pMP.at<float>(0),pMP.at<float>(1),pMP.at<float>(2));
        }
        glEnd();
    }

    void MapDrawer::DrawSurfels() {
        const vector<SurfelElement> &vSurfels = mpMap->mvLocalSurfels;
        const vector<SurfelElement> &vInactiveSurfels = mpMap->mvInactiveSurfels;

        if(vSurfels.empty() && vInactiveSurfels.empty())
            return;

        glPointSize(mPointSize/2);
        glBegin(GL_POINTS);

        for(auto& p : vSurfels){
            float norm = sqrt(p.r*p.r + p.g*p.g + p.b*p.b);
            glColor3f(p.r/norm, p.g/norm, p.b/norm);
            glVertex3f(p.px,p.py,p.pz);
        }

        for(auto& p : vInactiveSurfels){
            float norm = sqrt(p.r*p.r + p.g*p.g + p.b*p.b);
            glColor3f(p.r/norm, p.g/norm, p.b/norm);
            glVertex3f(p.px,p.py,p.pz);
        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
    {
        const float &w = mKeyFrameSize;
        const float h = w*0.75;
        const float z = w*0.6;

        const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f,0.0f,1.0f);
                glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }

        if(bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if(!vCovKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnId<vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame* pParent = vpKFs[i]->GetParent();
                if(pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
                {
                    if((*sit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w*0.75;
        const float z = w*0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if(!mCameraPose.empty())
        {
            cv::Mat Rwc(3,3,CV_32F);
            cv::Mat twc(3,1,CV_32F);
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
                twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
            }

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
            M.SetIdentity();
    }

} //namespace ORB_SLAM