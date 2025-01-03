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

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/sim3.h>
#include "Thirdparty/g2o/g2o/types/se3quat.h"

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>


namespace ORB_SLAM3 {


class EdgeSE3Barrier : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * \param T_ref        Reference pose (SE3Quat) whose translation is the center.
     * \param threshold    Max distance allowed from t_ref.
     * \param weight       Weight for the log barrier.
     * \param epsilon      Small offset to avoid log(0).
     */
    EdgeSE3Barrier(const g2o::SE3Quat& T_ref,
                   double threshold,
                   double weight,
                   double epsilon = 1e-3)
    : T_ref_(T_ref),
      threshold_(threshold),
      weight_(weight),
      epsilon_(epsilon)
    {
    }

    // The 'computeError' function calculates the barrier cost
    //   cost = - weight * log(threshold - dist + epsilon)
    // if dist < threshold, else it saturates to a large penalty
    void computeError() override
    {
        const auto* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat T_cur = v->estimate();
        
        Eigen::Vector3d t_ref = T_ref_.translation();
        Eigen::Vector3d t_cur = T_cur.translation();
        double dist = (t_cur - t_ref).norm();

        double gap = threshold_ - dist + epsilon_;  // "feasibility margin"

        if(gap <= 0.0) {
            // Already out of feasible region, set cost very high
            // to effectively push the solver away
            _error[0] = 1e6;
        } else {
            // Log barrier: cost = - w * log(gap)
            // The closer to threshold, the smaller 'gap' => cost grows large
            _error[0] = -weight_ * std::log(gap);
        }
    }

    // The 'linearizeOplus' function calculates the derivative of the cost
    //   wrt. the 6D SE3 state. We'll only differentiate in translation part.
    void linearizeOplus() override
    {
        _jacobianOplusXi.setZero();  // (1 x 6) matrix

        const auto* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat T_cur = v->estimate();

        Eigen::Vector3d t_ref = T_ref_.translation();
        Eigen::Vector3d t_cur = T_cur.translation();
        double dist = (t_cur - t_ref).norm();
        double gap = threshold_ - dist + epsilon_;

        if(gap <= 0.0) {
            // "Out-of-bounds" => derivative is basically zero or undefined
            // We'll just push the cost to huge. Let the solver try to step back.
            _jacobianOplusXi.setZero();
        } else {
            // derivative of cost = -w * d/dx( log(gap) ) = -w * ( 1 / gap ) * d(gap)/dx
            // gap = threshold - dist + epsilon
            // dist = norm(t_cur - t_ref)
            // d(dist)/d(t_cur) = ( t_cur - t_ref ) / dist
            if(dist < 1e-12) {
                // Avoid division by zero if t_cur == t_ref
                return;
            }
            double invGap = 1.0 / gap; 
            Eigen::Vector3d dir = (t_cur - t_ref) / dist;  // derivative of dist wrt t_cur
            // d(gap)/d(t_cur) = -d(dist)/d(t_cur) = -dir
            // => d(cost)/d(t_cur) = -w * 1/gap * (-dir) = w * (dir / gap)

            double coeff = weight_ * invGap;
            Eigen::Vector3d grad = coeff * dir;

            // In a SE3Expmap vertex, translation derivatives go in columns 3..5
            // (the first 3 columns are rotation-derivatives, the last 3 columns are translation)
            _jacobianOplusXi(0,3) = grad.x();
            _jacobianOplusXi(0,4) = grad.y();
            _jacobianOplusXi(0,5) = grad.z();
        }
    }

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

private:
    g2o::SE3Quat T_ref_;
    double threshold_;
    double weight_;
    double epsilon_;
};


class EdgeSE3Prior : public g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3Prior(const g2o::SE3Quat& T_prior);

    void computeError() override;
    void linearizeOplus() override;

    bool read(std::istream& is) override;
    bool write(std::ostream& os) const override;

private:
    g2o::SE3Quat T_prior_;
};

class EdgeSE3ProjectXYZOnlyPose: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera* pCamera;
};

class EdgeSE3ProjectXYZOnlyPoseToBody: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera* pCamera;

    g2o::SE3Quat mTrl;
};

class EdgeSE3ProjectXYZ: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2)>0.0);
    }

    virtual void linearizeOplus();

    GeometricCamera* pCamera;
};

class EdgeSE3ProjectXYZToBody: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2)>0.0;
    }

    virtual void linearizeOplus();

    GeometricCamera* pCamera;
    g2o::SE3Quat mTrl;
};

class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate = g2o::Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s*estimate());
    }

    GeometricCamera* pCamera1, *pCamera2;

    bool _fix_scale;
};

class EdgeSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap* v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->pCamera1->project(v1->estimate().map(v2->estimate()));
    }

    // virtual void linearizeOplus();

};

class EdgeInverseSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap* v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
    }

    // virtual void linearizeOplus();

};

}

#endif //ORB_SLAM3_OPTIMIZABLETYPES_H
