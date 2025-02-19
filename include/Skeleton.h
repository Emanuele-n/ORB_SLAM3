#ifndef SKELETON_H
#define SKELETON_H

#include <string>
#include <vector>
#include "Thirdparty/Sophus/sophus/sim3.hpp"

#include "Atlas.h"
#include "Map.h"

using namespace std;
namespace ORB_SLAM3 {

class Skeleton {
public:
    Skeleton(string &referenceCenterlinePath, Atlas* pAtlas);
    ~Skeleton();

    void Run();

    void SetCurvilinearAbscissa(double value);
    double GetCurvilinearAbscissa();

private:

    Atlas* mpAtlas;

    string mReferenceCenterlinePath;
    std::vector<std::vector<Sophus::SE3f>> mRefCenterlinePoses;
    std::mutex mMutexRefCenterlinePoses;

    void SetReferenceCenterline();
    std::vector<std::vector<Sophus::SE3f>> GetReferenceCenterline();
    std::vector<std::vector<Sophus::SE3f>> FindCandidateTrajectories();

    // Coming from encoder value 
    double mCurvilinearAbscissa; 
    std::mutex mMutexCurvilinearAbscissa;

    // Current pose on the real lungs
    Sophus::SE3f mCurPose;
    // prob a function to send the pose to the navigation through tcp ip
    
};

} // namespace ORB_SLAM3

#endif // SKELETON_H