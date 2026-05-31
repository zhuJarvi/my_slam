#include "interface/viewer.hpp"

namespace my_slam
{

Viewer::Viewer()
{
    viewer_running_ = false;
}

void Viewer::Close()
{
    viewer_running_ = false;
    if (viewer_thread_.joinable())
        viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap()
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    if (map_ != nullptr)
    {
        active_keyframes_ = map_->GetActiveKeyFrames();
        active_landmarks_ = map_->GetActiveMapPoints();
        map_updated_ = true;
    }
}

} // namespace my_slam
