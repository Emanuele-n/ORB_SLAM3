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


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;

class Optimizer
{
public:

    // Global Bundle Adjustment (Full BA): find all keyframes poses and all map points
    // Used in:
    //  - Map Initialization: Tracking::Track() -> MonocularInitialization() -> CreateInitialMapMonocular()
    //  - Loop Closing: LoopClosing::Run() -> RunGlobalBundleAdjustment()
    // void static GlobalBundleAdjustment(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0, const bool bRobust = true);
    // void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0, const bool bRobust = true);
    void static GlobalBundleAdjustment(bool withPatientData, bool withEncoder, const std::vector<std::vector<Sophus::SE3f>> &RefCenterlineFrames, Map* pMap, int nIterations=5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0, const bool bRobust = true);
    void static BundleAdjustment(bool withPatientData, bool withEncoder, const std::vector<std::vector<Sophus::SE3f>> &RefCenterlineFrames, const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0, const bool bRobust = true);

    // Local Bundle Adjustment: (Local BA) find recent keyframes poses and map points seen in those keyframes
    // Used in:
    //  - LocalMapping::Run()
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);
    
    // Local BA in welding area when two maps are merged
    // Used in:
    //  - LoopClosing::Run() -> MergeLocal()
    void static LocalBundleAdjustment(KeyFrame* pMainKF, vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);

    // Pose Optimization (Motion-only BA): find the current camera pose 
    // Used in:
    //  - Tracking::Track() -> TrackReferenceKeyFrame()
    //  - Tracking::Track() -> TrackWithMotionModel()
    //  - Tracking::Track() -> TrackLocalMap()
    //  - Tracking::Track() -> Relocalization()
    // int static PoseOptimization(Frame* pFrame); // EMA: commented out
    int static PoseOptimization(Frame *pFrame, bool withEncoder, const Sophus::SE3f &priorPose);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);
    void static OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                       vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs);


    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
