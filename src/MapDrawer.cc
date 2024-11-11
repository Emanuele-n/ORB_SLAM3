/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas), mTrackingInitialized(false)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::DrawTest()
{
    // Draw many huge test points
    glPointSize(100.0);  // Increased point size
    glBegin(GL_POINTS);
    glColor3f(0.0f,1.0f,0.0f); // Green color for points

    // Draw the transformed point
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 1.0, 1.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 1.0);
    glVertex3f(1.0, 1.0, 0.0);
    glVertex3f(1.0, 1.0, 1.0);

    glEnd();
}

bool MapDrawer::CheckInitialized()
{
    return mTrackingInitialized;
}

void MapDrawer::SetInitialized(std::vector<cv::Point3f> mvIniP3D, Sophus::SE3f Tcw, string& centerlineFramesPath)
{
    mInitTcw = Tcw;
    mvIniP3D = mvIniP3D;
    mCenterlineFramesPath = centerlineFramesPath;
    mTrackingInitialized = true;
}

void MapDrawer::DrawCenterline()
{
    // Read centerline frames from file
    std::ifstream file(mCenterlineFramesPath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << mCenterlineFramesPath << std::endl;
        return;
    }

    // Read each line of the file and draw the points
    std::string line;
    float x, y, z, tx, ty, tz, nx, ny, nz, bx, by, bz;
    char comma; // to consume the commas
    Eigen::Affine3f w_T_o; // Transformation from world to the first centerline point
    Eigen::Affine3f w_T_i; // Transformation from world to the i-th centerline point
    Eigen::Affine3f o_T_i; // Transformation from the first centerline point to the i-th point
    bool first = true;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        // Read the values from the line
        if (iss >> x >> comma >> y >> comma >> z >> comma >> tx >> comma >> ty >> comma >> tz >> comma >> nx >> comma >> ny >> comma >> nz >> comma >> bx >> comma >> by >> comma >> bz) {
            Eigen::Vector3f w_p_i(x, y, z);
            Eigen::Vector3f w_t_i(tx, ty, tz);
            Eigen::Vector3f w_n_i(nx, ny, nz);
            Eigen::Vector3f w_b_i(bx, by, bz);

            if (first){
                // Get the transformation w_T_o from world frame to the first centerline point (vectors are already normilized)
                // Construct the rotation matrix considering the tangent must be aligned with the z-axis
                // TODO: check better the order of the vectors, the forward positive direction is the tangent (z-axis) but I am not sure about the other two
                Eigen::Matrix3f R;
                R.col(2) = w_t_i;
                R.col(1) = w_n_i;
                R.col(0) = w_b_i;
                w_T_o.linear() = R;
                w_T_o.translation() = w_p_i;
                // cout << "w_T_o: " << w_T_o.matrix() << endl;
                first = false;
            }

            // // Coordinates of the i-th point wrt the first point
            // Eigen::Vector3f o_p_i = w_T_o.inverse() * w_p_i;
            // cout << "Point: " << w_p_i.transpose() << " -> " << o_p_i.transpose() << endl;

            // Get the Frenet-Serret frame at the i-th point
            Eigen::Matrix3f R;
            R.col(0) = w_t_i;
            R.col(1) = w_n_i;
            R.col(2) = w_b_i;
            w_T_i.linear() = R;
            w_T_i.translation() = w_p_i;

            // Get o_T_i transformation
            o_T_i = w_T_o.inverse() * w_T_i;
            Eigen::Vector3f o_p_i = o_T_i.translation();
            Eigen::Vector3f o_t_i = o_T_i.linear().col(0);
            Eigen::Vector3f o_n_i = o_T_i.linear().col(1);
            Eigen::Vector3f o_b_i = o_T_i.linear().col(2);
            // cout << "Point: " << w_p_i.transpose() << " -> " << o_p_i.transpose() << endl;

            // Draw both the original and the transformed point
            // glPointSize(5.0);  
            // glBegin(GL_POINTS);
            // glColor3f(1.0f,0.0f,0.0f); // Red color for points
            // glVertex3f(w_p_i(0), w_p_i(1), w_p_i(2));
            // glEnd();

            glPointSize(5.0);
            glBegin(GL_POINTS);
            glColor3f(1.0f,0.0f,0.0f); // Red color for points
            glVertex3f(o_p_i(0), o_p_i(1), o_p_i(2));
            glEnd();
            
            // Draw the Frenet-Serret frames (tangent, normal, binormal) as arrows
            float arrow_scale = 0.1f; // Scaling factor for arrow length

            // Draw the tangent vector (red)
            glBegin(GL_LINES);
            glColor3f(1.0f,0.0f,0.0f); 
            glVertex3f(o_p_i(0), o_p_i(1), o_p_i(2));
            glVertex3f(o_p_i(0) + arrow_scale*o_t_i(0), o_p_i(1) + arrow_scale*o_t_i(1), o_p_i(2) + arrow_scale*o_t_i(2));
            glEnd();

            // Draw the normal vector (green)
            glBegin(GL_LINES);
            glColor3f(0.0f,1.0f,0.0f); 
            glVertex3f(o_p_i(0), o_p_i(1), o_p_i(2));
            glVertex3f(o_p_i(0) + arrow_scale*o_n_i(0), o_p_i(1) + arrow_scale*o_n_i(1), o_p_i(2) + arrow_scale*o_n_i(2));
            glEnd();

            // Draw the binormal vector (blue)
            glBegin(GL_LINES);
            glColor3f(0.0f,0.0f,1.0f); 
            glVertex3f(o_p_i(0), o_p_i(1), o_p_i(2));
            glVertex3f(o_p_i(0) + arrow_scale*o_b_i(0), o_p_i(1) + arrow_scale*o_b_i(1), o_p_i(2) + arrow_scale*o_b_i(2));
            glEnd();

            // Draw the origin frame
            glBegin(GL_LINES);
            glColor3f(1.0f,0.0f,0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(1, 0, 0);
            glEnd();
            glBegin(GL_LINES);
            glColor3f(0.0f,1.0f,0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 1, 0);
            glEnd();
            glBegin(GL_LINES);
            glColor3f(0.0f,0.0f,1.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0, 1);
            glEnd();

        }
    }

    glEnd();

    // Close the file
    file.close();

}

void MapDrawer::DrawMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawOptLba)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat*)Twc.data());

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
                    }
                    else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f,0.0f,1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f,0.0f,1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

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

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

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

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i<4; i++) {
        M.m[4*i] = Twc(0,i);
        M.m[4*i+1] = Twc(1,i);
        M.m[4*i+2] = Twc(2,i);
        M.m[4*i+3] = Twc(3,i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0,3);
    MOw.m[13] = Twc(1,3);
    MOw.m[14] = Twc(2,3);
}
} //namespace ORB_SLAM
