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

#include "Atlas.h"
#include "Viewer.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "spline.h"

// Helper function to create Gaussian kernel
std::vector<double> gaussianKernel(int size, double sigma) {
    std::vector<double> kernel(size);
    double sum = 0.0;
    int mean = size/2;
    
    for(int x = 0; x < size; x++) {
        kernel[x] = exp(-0.5 * pow((x - mean)/sigma, 2.0));
        sum += kernel[x];
    }
    
    // Normalize the kernel
    for(int x = 0; x < size; x++) {
        kernel[x] /= sum;
    }
    
    return kernel;
}

namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::SetRefCenterline(string& refCenterlineFramesPath)
{   
    // Protect with mutex
    unique_lock<mutex> lock(mMutexRefCenterlineFrames);

    // Read centerline frames from file
    std::ifstream file(refCenterlineFramesPath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << refCenterlineFramesPath << std::endl;
        return;
    }

    // Clear previous frames
    mRefCenterlineFrames.clear();

    // Read each line of the file and process the points
    std::string line;
    float x, y, z, tx, ty, tz, nx, ny, nz, bx, by, bz;
    char comma; // to consume the commas
    Sophus::SE3f w_T_o; // Transformation from world to the first centerline point
    Sophus::SE3f w_T_i; // Transformation from world to the i-th centerline point
    Sophus::SE3f o_T_i; // Transformation from the first centerline point to the i-th point
    bool first = true;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        // Read the values from the line
        if (iss >> x >> comma >> y >> comma >> z >> comma >> tx >> comma >> ty >> comma >> tz >> comma >> nx >> comma >> ny >> comma >> nz >> comma >> bx >> comma >> by >> comma >> bz) {
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

            if (first){
                w_T_o = Sophus::SE3f(R, w_p_i);
                first = false;
            }

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
            o_T_i = Sophus::SE3f(Ry_inv, Eigen::Vector3f(0, 0, 0)) * o_T_i * Sophus::SE3f(Ry, Eigen::Vector3f(0, 0, 0));
            
            // Save the current o_T_i in mRefCenterlineFrames
            mRefCenterlineFrames.push_back(o_T_i);
        }
    }
    // Close the file
    cout << "Set reference centerline in Atlas with " << mRefCenterlineFrames.size() << " frames" << endl;
    file.close();
}

std::vector<Sophus::SE3f> Atlas::GetRefCenterlineFrames()
{
    unique_lock<mutex> lock(mMutexRefCenterlineFrames);
    return mRefCenterlineFrames;
}

double Atlas::GetCurvilinearAbscissa(Eigen::Vector3d& Ow_d)
{   
    bool debug = false;
    if (debug) cout << "Getting curvilinear abscissa" << endl;
    if (debug) cout << "Camera center: " << Ow_d.transpose() << endl;
    // Check if Ow_d is valid
    if (Ow_d.hasNaN()) {
        std::cerr << "Invalid Ow_d received." << std::endl;
        return 0.0;
    }

    // Get all keyframes
    Map* pMap = GetCurrentMap();
    if (debug) cout << "Got current map" << endl;
    if(!pMap)
    {
        cout << "Current map is empty" << endl;
        return 0.0;
    }

    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    if(vpKFs.empty())
    {
        cout << "No keyframes in the map" << endl;
        return 0.0;
    }

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
    {
        cout << "Closest keyframe not found" << endl;
        return 0.0;
    }
    if (debug) cout << "Found closest key frame" << endl;

    // Find path to origin through spanning tree
    std::vector<KeyFrame*> path;
    KeyFrame* current = closestKF;

    // Follow parent pointers until we reach a keyframe with no parent (origin)
    while(current) {
        path.push_back(current);
        current = current->GetParent();
    }
    if (debug) cout << "Done building path" << endl;

    // Compute curvilinear abscissa by summing distances between consecutive keyframes
    double s = 0.0;
    for(int i = path.size()-1; i > 0; i--) {
        Eigen::Vector3f pos1 = path[i]->GetCameraCenter();
        Eigen::Vector3f pos2 = path[i-1]->GetCameraCenter();
        s += (pos2 - pos1).norm();
    }
    if (debug) cout << "Done computing s" << endl;

    // Add distance from closest keyframe to current position
    s += minDist;

    // TODO: Reduce the total length to take into account the non smoothness of the path based on the number of points in the path

    return s;
}

void Atlas::CreateNewMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Change to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
{
    //Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    for(size_t i=0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera* pCam_i = mvpCameras[i];
        if(!pCam) std::cout << "Not pCam" << std::endl;
        if(!pCam_i) std::cout << "Not pCam_i" << std::endl;
        if(pCam->GetType() != pCam_i->GetType())
            continue;

        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if(((Pinhole*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            if(((KannalaBrandt8*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if(bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else{
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras()
{
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    for(Map* pMi : mvpBackupMaps)
    {
        if(!pMi || pMi->IsBad())
            continue;

        if(pMi->GetAllKeyFrames().size() == 0) {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

void Atlas::PostLoad()
{
    map<unsigned int,GeometricCamera*> mpCams;
    for(GeometricCamera* pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for(Map* pMi : mvpBackupMaps)
    {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* pMap_i : mspMaps)
    {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes()
{
    map<long unsigned int, KeyFrame*> mpIdKFs;
    for(Map* pMap_i : mvpBackupMaps)
    {
        vector<KeyFrame*> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for(KeyFrame* pKF_j_Mi : vpKFs_Mi)
        {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

} //namespace ORB_SLAM3
