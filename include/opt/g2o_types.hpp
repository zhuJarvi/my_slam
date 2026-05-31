#pragma once
#ifndef __G2O_TYPES_HPP__
#define __G2O_TYPES_HPP__

#include "common.hpp"
#include "stereo_vo/imu.hpp"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace my_slam
{
    class VertexXYZ;

    class VertexPose : public g2o::BaseVertex<6, SE3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override { _estimate = SE3(); }

        void oplusImpl(const double *update) override
        {
            Vec6 update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = SE3::exp(update_eigen) * _estimate;
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }
    };

    class VertexXYZ : public g2o::BaseVertex<3, Vec3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override { _estimate = Vec3::Zero(); }

        void oplusImpl(const double *update) override
        {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }
    };

    struct NavState
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        SE3 pose_ = SE3();
        Vec3 vel_ = Vec3::Zero();
        Vec3 ba_ = Vec3::Zero();
        Vec3 bg_ = Vec3::Zero();
    };

    class VertexNavState : public g2o::BaseVertex<15, NavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void setToOriginImpl() override { _estimate = NavState(); }

        void oplusImpl(const double *update) override
        {
            Vec6 dp;
            for (int i = 0; i < 6; ++i)
                dp[i] = update[i];
            _estimate.pose_ = SE3::exp(dp) * _estimate.pose_;
            for (int i = 0; i < 3; ++i)
            {
                _estimate.vel_[i] += update[6 + i];
                _estimate.ba_[i] += update[9 + i];
                _estimate.bg_[i] += update[12 + i];
            }
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }
    };

    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K) : _pos3d(pos), _K(K) {}

        void computeError() override
        {
            const VertexPose *v = static_cast<const VertexPose *>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3 pos_pixel = _K * (T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        void linearizeOplus() override
        {
            const VertexPose *v = static_cast<const VertexPose *>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3 pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv,
                0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2,
                -fy * X * Y * Zinv2, -fy * X * Zinv;
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }

    private:
        Vec3 _pos3d;
        Mat33 _K;
    };

    class EdgeProjection : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : _K(K), _cam_ext(cam_ext) {}

        void computeError() override
        {
            const VertexPose *v0 = static_cast<const VertexPose *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<const VertexXYZ *>(_vertices[1]);
            SE3 T = v0->estimate();
            Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        void linearizeOplus() override
        {
            const VertexPose *v0 = static_cast<const VertexPose *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<const VertexXYZ *>(_vertices[1]);
            SE3 T = v0->estimate();
            Vec3 pw = v1->estimate();
            Vec3 pos_cam = _cam_ext * T * pw;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv,
                0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2,
                -fy * X * Y * Zinv2, -fy * X * Zinv;
            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }

    private:
        Mat33 _K;
        SE3 _cam_ext;
    };

    class EdgeIMUPreint : public g2o::BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void computeError() override
        {
            const auto *v0 = static_cast<const VertexNavState *>(_vertices[0]);
            const auto *v1 = static_cast<const VertexNavState *>(_vertices[1]);
            const NavState &s0 = v0->estimate();
            const NavState &s1 = v1->estimate();

            SE3 pose_pred = s0.pose_ * SE3(_measurement.delta_q(), _measurement.delta_p());
            Vec3 vel_pred = s0.vel_ + _measurement.delta_v();
            Vec6 pose_err = (pose_pred.inverse() * s1.pose_).log();

            _error.setZero();
            _error.block<6, 1>(0, 0) = pose_err;
            _error.block<3, 1>(6, 0) = vel_pred - s1.vel_;
            _error.block<3, 1>(9, 0) = s1.ba_ - s0.ba_;
            _error.block<3, 1>(12, 0) = s1.bg_ - s0.bg_;
        }

        void linearizeOplus() override
        {
            _jacobianOplusXi.setIdentity();
            _jacobianOplusXj = -Mat1515::Identity();
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }
    };

    class EdgeProjectionNavState : public g2o::BaseBinaryEdge<2, Vec2, VertexNavState, VertexXYZ>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionNavState(const Mat33 &K, const SE3 &cam_ext) : _K(K), _cam_ext(cam_ext) {}

        void computeError() override
        {
            const VertexNavState *v0 = static_cast<const VertexNavState *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<const VertexXYZ *>(_vertices[1]);
            SE3 T = v0->estimate().pose_;
            Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        void linearizeOplus() override
        {
            const VertexNavState *v0 = static_cast<const VertexNavState *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<const VertexXYZ *>(_vertices[1]);
            SE3 T = v0->estimate().pose_;
            Vec3 pw = v1->estimate();
            Vec3 pos_cam = _cam_ext * T * pw;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv,
                0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2,
                -fy * X * Y * Zinv2, -fy * X * Zinv;
            _jacobianOplusXi.setZero();
            _jacobianOplusXi.block<2, 6>(0, 0) = J;
            _jacobianOplusXj = J.block<2, 3>(0, 0) * _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        bool read(std::istream &in) override { return true; }
        bool write(std::ostream &out) const override { return true; }

    private:
        Mat33 _K;
        SE3 _cam_ext;
    };
}

#endif // __G2O_TYPES_HPP__
