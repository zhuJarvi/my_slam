#include "stereo_vo/struct_base/frame.hpp"
#include "stereo_vo/struct_base/feature.hpp"
#include "stereo_vo/struct_base/mappoint.hpp"

namespace my_slam
{
    void Frame::SetKeyFrame()
    {
        static std::atomic<unsigned long> keyframe_factory_id{0};
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id.fetch_add(1, std::memory_order_relaxed);
    }

    Frame::Ptr Frame::CreateFrame()
    {
        static long factory_id = 0;
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }
}