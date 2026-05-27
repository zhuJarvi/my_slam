#pragma once
#ifndef __IMU_HPP__
#define __IMU_HPP__

#include "common.hpp"

namespace my_slam
{
    struct IMUData
    {
        double timestamp = 0.0;
        Vec3 acc = Vec3::Zero();
        Vec3 gyro = Vec3::Zero();
    };

    /**
     * Preintegration skeleton for IMU measurements.
     * We'll fill in integration logic later; for now keep an API used by frontend/backend.
     */
    class IMUPreintegrator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        IMUPreintegrator() = default;

        void Reset(const Vec3 &bias_acc, const Vec3 &bias_gyro)
        {
            bias_acc_ = bias_acc;
            bias_gyro_ = bias_gyro;
            dt_sum_ = 0.0;
            delta_p_.setZero();
            delta_v_.setZero();
            delta_q_ = SO3();
            covariance_.setZero();
        }

        void IntegrateMeasurement(const Vec3 &acc, const Vec3 &gyro, double dt)
        {
            // remove biases
            Vec3 una_acc = acc - bias_acc_;
            Vec3 una_gyro = gyro - bias_gyro_;

            // integrate rotation: delta_q = delta_q * Exp(omega*dt)
            SO3 dq = SO3::exp(una_gyro * dt);
            delta_q_ = delta_q_ * dq;

            // update delta_v and delta_p (assume body-frame measurements)
            Vec3 acc_world = delta_q_ * una_acc; // rotate to world approximation
            delta_p_ += delta_v_ * dt + 0.5 * acc_world * dt * dt;
            delta_v_ += acc_world * dt;

            // simple covariance growth model for downstream weighting
            double acc_var = 0.05 * 0.05;
            double gyro_var = 0.01 * 0.01;
            double bias_acc_var = 0.0001 * 0.0001;
            double bias_gyro_var = 0.0001 * 0.0001;
            covariance_.block<3, 3>(0, 0).diagonal().array() += gyro_var * dt;
            covariance_.block<3, 3>(3, 3).diagonal().array() += acc_var * dt;
            covariance_.block<3, 3>(6, 6).diagonal().array() += acc_var * dt * dt;
            covariance_.block<3, 3>(9, 9).diagonal().array() += bias_acc_var * dt;
            covariance_.block<3, 3>(12, 12).diagonal().array() += bias_gyro_var * dt;

            dt_sum_ += dt;
        }

        SO3 delta_q() const { return delta_q_; }
        Vec3 delta_p() const { return delta_p_; }
        Vec3 delta_v() const { return delta_v_; }
        Mat1515 covariance() const { return covariance_; }

        double dt_sum() const { return dt_sum_; }

    private:
        Vec3 bias_acc_ = Vec3::Zero();
        Vec3 bias_gyro_ = Vec3::Zero();

        double dt_sum_ = 0.0;
        Vec3 delta_p_ = Vec3::Zero();
        Vec3 delta_v_ = Vec3::Zero();
        SO3 delta_q_ = SO3();
        Mat1515 covariance_ = Mat1515::Zero();
    };
}

#endif // __IMU_HPP__
