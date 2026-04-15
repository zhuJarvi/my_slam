#pragma once
#ifndef __FRAME_HPP__
#define __FRAME_HPP__

#include "common.hpp"
#include "interface/camera.hpp"

namespace my_slam
{
    struct Feature;
    struct MapPoint;

    /**
     * 帧
     * 每一帧分配独立id，关键帧分配关键帧ID
     */
    struct Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;          // id of this frame
        unsigned long keyframe_id_ = 0; // id of key frame
        bool is_keyframe_ = false;      // 是否为关键帧
        double time_stamp_;             // 时间戳，暂不使用8
        SE3 pose_;                      // Tcw 形式Pose
        std::mutex pose_mutex_;         // Pose数据锁
        cv::Mat left_img_, right_img_;  // stereo images

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;

    public: // data members
        Frame() {}

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
            : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

        // set and get pose, thread safe
        SE3 GetPose()
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const SE3 &pose)
        {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        /// 设置关键帧并分配并键帧id
        void SetKeyFrame();

        /// 工厂构建模式，分配id
        static std::shared_ptr<Frame> CreateFrame();
    };
}

#endif // __FRAME_HPP__
