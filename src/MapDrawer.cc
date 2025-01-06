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

void MapDrawer::SetInitialized(std::vector<cv::Point3f> mvIniP3D, Sophus::SE3f Tcw, string& refCenterlineFramesPath)
{
    mInitTcw = Tcw;
    mvIniP3D = mvIniP3D;
    SetRefCenterline(refCenterlineFramesPath);
    mTrackingInitialized = true;
}

void MapDrawer::DrawOrigin()
{
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

void MapDrawer::DrawCameraTrajectory()
{
    Map* pMap = mpAtlas->GetCurrentMap();
    if(!pMap)
        return;

    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    if(vpKFs.empty())
        return;

    glLineWidth(2.0f);
    glColor3f(1.0f, 0.0f, 0.0f); // Red color for the trajectory

    glBegin(GL_LINE_STRIP);
    for(size_t i = 0; i < vpKFs.size(); ++i)
    {
        KeyFrame* pKF = vpKFs[i];
        if(!pKF || pKF->isBad())
            continue;

        Eigen::Vector3f Ow = pKF->GetCameraCenter();
        glVertex3f(Ow[0], Ow[1], Ow[2]);
    }
    glEnd();
}

std::vector<std::vector<Sophus::SE3f>> MapDrawer::GetRefCenterlineFrames()
{
    unique_lock<mutex> lock(mMutexRefCenterlineFrames);
    return mRefCenterlineFrames;
}

void MapDrawer::DrawRefCenterline()
{
    // Access the centerline branches (vector of vectors)
    std::vector<std::vector<Sophus::SE3f>> refCenterlineBranches = GetRefCenterlineFrames();

    // If no branches are stored, return
    if (refCenterlineBranches.empty())
        return;

    glPointSize(5.0f);

    // Iterate over each branch
    for (const auto& branch : refCenterlineBranches)
    {
        if (branch.empty())
            continue; // Skip empty branch

        // Draw all frames' points in the current branch
        glBegin(GL_POINTS);
        glColor3f(0.0f, 0.0f, 1.0f);  // Blue points
        for (const auto& frame : branch)
        {
            Eigen::Vector3f pos = frame.translation();
            glVertex3f(pos[0], pos[1], pos[2]);
        }
        glEnd();

        // Draw coordinate frames for ~10% of the points in this branch
        glLineWidth(2.0f);
        // for (const auto& frame : branch)
        // {
        //     // Extract a random number between 0 and 1
        //     float random = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        //     if (random < 0.1f)  // ~10% chance
        //     {
        //         Eigen::Vector3f pos = frame.translation();
        //         Eigen::Matrix3f rot = frame.rotationMatrix();

        //         // Draw tangent vector (red)
        //         glBegin(GL_LINES);
        //         glColor3f(1.0f, 0.0f, 0.0f);
        //         glVertex3f(pos[0], pos[1], pos[2]);
        //         Eigen::Vector3f tangent = pos + 0.02f * rot.col(0);
        //         glVertex3f(tangent[0], tangent[1], tangent[2]);
        //         glEnd();

        //         // Draw normal vector (green)
        //         glBegin(GL_LINES);
        //         glColor3f(0.0f, 1.0f, 0.0f);
        //         glVertex3f(pos[0], pos[1], pos[2]);
        //         Eigen::Vector3f normal = pos + 0.02f * rot.col(1);
        //         glVertex3f(normal[0], normal[1], normal[2]);
        //         glEnd();

        //         // Draw binormal vector (blue)
        //         glBegin(GL_LINES);
        //         glColor3f(0.0f, 0.0f, 1.0f);
        //         glVertex3f(pos[0], pos[1], pos[2]);
        //         Eigen::Vector3f binormal = pos + 0.02f * rot.col(2);
        //         glVertex3f(binormal[0], binormal[1], binormal[2]);
        //         glEnd();
        //     }
        // }
    }
}

void MapDrawer::DrawCandidateFrame(Sophus::SE3f Tfw)
{
    // Draw the candidate frame
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f pos = Tfw.translation();
    glVertex3f(pos[0], pos[1], pos[2]);
    glEnd();
}

void MapDrawer::SetRefCenterline(string& refCenterlineFramesPath)
{
    // Protect with mutex
    unique_lock<mutex> lock(mMutexRefCenterlineFrames);

    // Clear previous frames
    mRefCenterlineFrames.clear();

    int branchIndex = 1;
    while (true)
    {
        // Build the full filename: e.g., <input_folder>/1.txt, b2.txt, etc.
        std::string filePath = refCenterlineFramesPath + "/b" + std::to_string(branchIndex) + ".txt";
        cout << "Reading centerline file: " << filePath << endl;
        
        // Try to open the file
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "No more centerline files found after index " << branchIndex << ". Stopping." << std::endl;
            break;
        }

        // A temporary container for this branch’s frames
        std::vector<Sophus::SE3f> currentBranchFrames;

        // Read each line of the file and process the points
        std::string line;
        float x, y, z, tx, ty, tz, nx, ny, nz, bx, by, bz;
        char comma; // to consume the commas
        Sophus::SE3f w_T_i; // Transformation from world to the i-th centerline point (written in the centerline data file)
        Sophus::SE3f o_T_i; // Transformation from the first centerline point to the i-th point (to be found)
        Sophus::SE3f w_T_o; // Transformation from world to the first centerline point (to transform from world to origin)
        // World frame in this case is referred to the world where the CAD model is built and the centerline is extracted
        // while the origin frame is the world frame in the viewer and SLAM system
        // So the goal is to write the reference centerline in the origin frame
        bool first = true;

        while (std::getline(file, line)) {
            std::istringstream iss(line);

            // Read the values from the line
            if (iss >> x >> comma >> y >> comma >> z >> comma >> tx >> comma >> ty >> comma >> tz 
                    >> comma >> nx >> comma >> ny >> comma >> nz >> comma >> bx >> comma >> by >> comma >> bz) 
            {
                Eigen::Vector3f w_p_i(x, y, z);
                Eigen::Vector3f w_t_i(tx, ty, tz);
                Eigen::Vector3f w_n_i(nx, ny, nz);
                Eigen::Vector3f w_b_i(bx, by, bz);

                // Construct rotation matrix as it is written in the file: t forward, n down, b left
                Eigen::Matrix3f R;
                R.col(0) = w_t_i;
                R.col(1) = w_n_i;
                R.col(2) = w_b_i;

                // Check determinant and adjust if necessary
                if (R.determinant() < 0) {
                    std::cerr << "Determinant is negative. Adjusting rotation matrix." << std::endl;
                    R.col(2) *= -1.0f;
                }

                // Verify orthogonality
                Eigen::Matrix3f RtR = R.transpose() * R;
                Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
                Eigen::Matrix3f error = RtR - I;
                float maxError = error.cwiseAbs().maxCoeff();
                if (maxError > 1e-6) {
                    std::cerr << "Rotation matrix is not orthogonal enough. Max error: " << maxError << std::endl;
                    continue; // Skip this frame
                }

                // Find the transformation from world to the first centerline frame
                if (first){
                    w_T_o = Sophus::SE3f(R, w_p_i);
                    first = false;
                }
                
                // Compute w_T_i transformation as written in the centerline file
                w_T_i = Sophus::SE3f(R, w_p_i);

                // Compute o_T_i transformation
                o_T_i = w_T_o.inverse() * w_T_i;

                // Rotate 90 degrees around n axis to match camera convention: 
                // b forward, t right, n down -> z forward, x right, y down
                Eigen::Matrix3f Ry;
                Ry << 0, 0, 1,
                      0, 1, 0,
                     -1, 0, 0;

                Eigen::Matrix3f Ry_inv = Ry.transpose();

                // Build the transformation to match the camera convention
                o_T_i = Sophus::SE3f(Ry_inv, Eigen::Vector3f(0, 0, 0)) 
                        * o_T_i 
                        * Sophus::SE3f(Ry, Eigen::Vector3f(0, 0, 0));

                // Save the current o_T_i in currentBranchFrames
                currentBranchFrames.push_back(o_T_i);
            }
        }

        // Close this branch file
        file.close();

        // Now add this branch’s frames to the main container
        mRefCenterlineFrames.push_back(currentBranchFrames);

        std::cout << "Set reference centerline for branch " << branchIndex 
                  << " in MapDrawer with " << currentBranchFrames.size() << " frames" 
                  << std::endl;

        // Go to the next branch index
        branchIndex++;
    }

    // Optional: You can print how many total branches got loaded
    std::cout << "Total branches loaded: " << mRefCenterlineFrames.size() << std::endl;
}

void MapDrawer::DrawTrajCenterline()
{
    // Get current position
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }
    Eigen::Vector3f Ow = Twc.block<3, 1>(0, 3);
    Eigen::Vector3d Ow_d = Ow.cast<double>();

    // Get all keyframes
    Map* pMap = mpAtlas->GetCurrentMap();
    if(!pMap)
        return;

    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    if(vpKFs.empty())
        return;

    // Find the keyframe closest to current position
    KeyFrame* closestKF = nullptr;
    double minDist = std::numeric_limits<double>::max();
    for(const auto& pKF : vpKFs) {
        Eigen::Vector3f kfPos = pKF->GetCameraCenter();
        double dist = (kfPos.cast<double>() - Ow_d).norm();
        if(dist < minDist) {
            minDist = dist;
            closestKF = pKF;
        }
    }

    if(!closestKF)
        return;

    // Find path to origin through spanning tree
    std::vector<KeyFrame*> path;
    KeyFrame* current = closestKF;

    // Follow parent pointers until we reach a keyframe with no parent (origin)
    while(current) {
        path.push_back(current);
        current = current->GetParent();
    }

    // Draw the path
    if(path.size() > 1) {
        glLineWidth(2.0f);
        glColor3f(0.0f, 1.0f, 0.0f); // Green path
        glBegin(GL_LINE_STRIP);
        
        for(const auto& pKF : path) {
            Eigen::Vector3f pos = pKF->GetCameraCenter();
            glVertex3f(pos[0], pos[1], pos[2]);
        }
        
        glEnd();
    }

    // Draw all the key frames and all the key frames in the path
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f); // Red color for all keyframes
    for(const auto& pKF : vpKFs) {
        Eigen::Vector3f pos = pKF->GetCameraCenter();
        glVertex3f(pos[0], pos[1], pos[2]);
    }
    glEnd();

    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 0.0f, 1.0f); // Blue color for path keyframes
    for(const auto& pKF : path) {
        Eigen::Vector3f pos = pKF->GetCameraCenter();
        glVertex3f(pos[0], pos[1], pos[2]);
    }
    glEnd();

    // Draw the current position
    glPointSize(20.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow color for current position
    glVertex3f(Ow[0], Ow[1], Ow[2]);
    glEnd();

    
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
    // const float &w = mCameraSize/10;
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

//     glPopMatrix();

    // Draw camera frame as done for the reference frames
    // Get cTw 
    Eigen::Matrix4f TwcEigen;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            TwcEigen(i,j) = Twc.m[i+4*j];
    Eigen::Matrix4f Tcw = TwcEigen.inverse();

    // if ((TwcEigen - Eigen::Matrix4f::Identity()).norm() > 1e-6) {
    //     std::cout << "Twc:\n" << TwcEigen << std::endl;
    //     std::cout << "Tcw:\n" << Tcw << std::endl;
    // }

    // Extract position and rotation
    // Eigen::Vector3f pos = Tcw.block<3, 1>(0, 3);
    // Eigen::Matrix3f rot = Tcw.block<3, 3>(0, 0);
    Eigen::Vector3f pos = TwcEigen.block<3, 1>(0, 3);
    Eigen::Matrix3f rot = TwcEigen.block<3, 3>(0, 0);

    // Draw xc (right) vector (red)
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(pos[0], pos[1], pos[2]);
    Eigen::Vector3f xc = pos + 0.02f * rot.col(0);
    glVertex3f(xc[0], xc[1], xc[2]);
    glEnd();

    // Draw yc (down) vector (green)
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(pos[0], pos[1], pos[2]);
    Eigen::Vector3f yc = pos + 0.02f * rot.col(1);
    glVertex3f(yc[0], yc[1], yc[2]);
    glEnd();

    // Draw zc (forward) vector (blue)
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(pos[0], pos[1], pos[2]);
    Eigen::Vector3f zc = pos + 0.02f * rot.col(2);
    glVertex3f(zc[0], zc[1], zc[2]);
    glEnd();

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
