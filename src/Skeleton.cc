#include "Skeleton.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>

namespace ORB_SLAM3 {

Skeleton::Skeleton(string &referenceCenterlinePath, Atlas* pAtlas)
    : mReferenceCenterlinePath(referenceCenterlinePath), mpAtlas(pAtlas) {
    // Load the reference centerline poses
    SetReferenceCenterline();
}

// TODO
Skeleton::~Skeleton() {
    // Cleanup resources if needed.
}

// TODO
void Skeleton::Run() {
    // Find candidate trajectories
    std::vector<std::vector<Sophus::SE3f>> candidateTrajectories = FindCandidateTrajectories();

    // Get the current trajectory (coming from slam) 
    // TODOE: this must be the trajectory from the current pose to the origin to the complete trajectory
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap) {
        std::cerr << "No active map found." << std::endl;
        return;
    }
    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();
    if (vpKFs.empty()) {
        std::cerr << "No keyframes found." << std::endl;
        return;
    }
    std::vector<Sophus::SE3f> currentTrajectory;
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame* pKF = vpKFs[i];
        Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
        Sophus::SE3f pose(Twc);
        currentTrajectory.push_back(pose);
    }

    // Align current trajectory to each reference centerline

    // Find the one with the smallest ATE

}

void Skeleton::SetCurvilinearAbscissa(double value) {
    std::unique_lock<std::mutex> lock(mMutexCurvilinearAbscissa);
    mCurvilinearAbscissa = value;
}

double Skeleton::GetCurvilinearAbscissa() {
    std::unique_lock<std::mutex> lock(mMutexCurvilinearAbscissa);
    return mCurvilinearAbscissa;
}

void Skeleton::SetReferenceCenterline() {

    // Assume the centerlines are saved as a series of pose Twc in TUM format: timestamp px py pz qx qy qz qw
    // in a single .txt file for each branch
    
    // Protect with mutex
    std::unique_lock<std::mutex> lock(mMutexRefCenterlinePoses);

    // Clear any existing reference centerline poses.
    mRefCenterlinePoses.clear();

    bool isDebug = true;

    int branchId = 1;
    while (true){
        // Build the full name: e.g. <input_folder>/b1.txt , <input_folder>/b2.txt etc.
        std::string branchFileName = mReferenceCenterlinePath + "/b" + std::to_string(branchId) + ".txt";
        if (isDebug) std::cout << "Reading branch file: " << branchFileName << std::endl;

        // Try to open the file.
        std::ifstream branchFile(branchFileName);
        if (!branchFile.is_open()) {
            if (isDebug) std::cout << "No more centerline found after index: " << branchId << std::endl; 
            break;
        }

        // Temporary container for the current branch poses
        std::vector<Sophus::SE3f> branchPoses;

        // Read the file line by line.
        std::string line;
        while (std::getline(branchFile, line)) {
            std::istringstream iss(line);
            double timestamp, px, py, pz, qx, qy, qz, qw;
            if (!(iss >> timestamp >> px >> py >> pz >> qx >> qy >> qz >> qw)) {
                if (isDebug) std::cout << "Error reading line: " << line << std::endl;
                break;
            }

            // Create a SE3 pose from the read values.
            Eigen::Quaternionf q(qw, qx, qy, qz);
            Eigen::Vector3f t(px, py, pz);
            Sophus::SE3f pose(q, t);

            // Add the pose to the branch poses.
            branchPoses.push_back(pose);
        }

        // Close this branch file.
        branchFile.close();

        // Add the branch poses to the reference centerline poses.
        mRefCenterlinePoses.push_back(branchPoses);

        if (isDebug) std::cout << "Branch " << branchId << " poses: " << branchPoses.size() << std::endl;

        // Move to the next branch.
        branchId++;
    }

    // Print the number of branches and poses.
    if (isDebug) std::cout << "Total branches centerline loaded: " << mRefCenterlinePoses.size() << std::endl;

}

std::vector<std::vector<Sophus::SE3f>> Skeleton::GetReferenceCenterline() {
    std::unique_lock<std::mutex> lock(mMutexRefCenterlinePoses);
    return mRefCenterlinePoses;
}

std::vector<std::vector<Sophus::SE3f>> Skeleton::FindCandidateTrajectories() {

    bool isDebug = true;

    std::vector<std::vector<Sophus::SE3f>> candidateTrajectories;

    // Get the reference centerline poses
    std::vector<std::vector<Sophus::SE3f>> refCenterlinePoses = GetReferenceCenterline();
    if (refCenterlinePoses.empty()) {
        std::cerr << "No reference centerline poses found." << std::endl;
        return candidateTrajectories;
    }

    // Check if all branches are empty
    bool allBranchesEmpty = true;
    for (auto &branchPoses : refCenterlinePoses) {
        if (!branchPoses.empty()) {
            allBranchesEmpty = false;
            break;
        }
    }
    if (allBranchesEmpty) {
        std::cerr << "All branches are empty." << std::endl;
        return candidateTrajectories;
    }

    // Get the current curvilinear abscissa
    double curvilinearAbscissa = GetCurvilinearAbscissa();

    // Loop over each branch and find the candidate pose for each 
    for (size_t b = 0; b < refCenterlinePoses.size(); b++) {
        std::vector<Sophus::SE3f> &branchPoses = refCenterlinePoses[b];
        if (branchPoses.empty()) {
            if (isDebug) std::cout << "Branch " << b << " is empty." << std::endl;
            continue;
        }

        // Find the candidate pose for this branch by projecting the c.a. onto the centerline of the branch
        Sophus::SE3f candidatePose;
        size_t candidatePoseIndex = 0;
        double minDist = std::numeric_limits<double>::max();
        double s_i = 0.0;
        bool candidateFound = false;

        for (size_t i = 1; i < branchPoses.size(); i++) {
            Eigen::Vector3d p1 = branchPoses[i-1].translation().cast<double>();
            Eigen::Vector3d p2 = branchPoses[i].translation().cast<double>();

            // Compute the distance between two consecutive points
            double dist = (p2 - p1).norm();

            // Add the distance to the current s_i
            s_i += dist;

            // Compute the difference between the current c.a. and the real one
            double diff = std::abs(curvilinearAbscissa - s_i);
            if (diff < minDist) {
                minDist = diff;
                candidatePose = branchPoses[i];
                candidatePoseIndex = i;
                candidateFound = true;
            }
            // TODOE: break if the distance is increasing
        }
        if (!candidateFound) {
            std::cerr << "No candidate pose found for branch: " << b << std::endl;
            continue;
        }
        else{
            // Build the candidate trajectory for this branch taking all the poses from the beginning to the candidate pose
            std::vector<Sophus::SE3f> candidateTrajectory;
            for (size_t i = 0; i <= candidatePoseIndex; i++) {
                candidateTrajectory.push_back(branchPoses[i]);
            }
            candidateTrajectories.push_back(candidateTrajectory);
        }
    }

    return candidateTrajectories;
}


} // namespace ORB_SLAM3