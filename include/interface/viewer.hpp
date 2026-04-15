#pragma once
#ifndef __VIEWER_HPP__
#define __VIEWER_HPP__

#include <thread>
#include "pangolin/pangolin.h"

#include "common.hpp"
#include "stereo_vo/struct_base/frame.hpp"
#include "stereo_vo/struct_base/mappoint.hpp"
#include "stereo_vo/struct_base/feature.hpp"
#include "stereo_vo/map.hpp"

namespace my_slam
{
    /**
     * 可视化
     */
    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) { map_ = map; }

        void Close();

        // 增加一个当前帧
        void AddCurrentFrame(Frame::Ptr current_frame);

        // 更新地图
        void UpdateMap();

    private:
        void ThreadLoop();

        void DrawFrame(Frame::Ptr frame, const float *color);

        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

        /// plot the features in current frame into an image
        cv::Mat PlotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;
    };
}

#endif // __VIEWER_HPP__
