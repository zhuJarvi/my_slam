#pragma once
#ifndef __DATASET_HPP__
#define __DATASET_HPP__

#include "common.hpp"
#include "interface/camera.hpp"
#include "stereo_vo/imu.hpp"
#include "stereo_vo/struct_base/frame.hpp"

namespace my_slam
{
    /**
     * 数据集读取
     * 构造时传入配置文件路径，配置文件的dataset_dir为数据集路径
     * Init之后可获得相机和下一帧图像
     */
    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;
        Dataset(const std::string &dataset_path)
        {
            dataset_path_ = dataset_path;
        }
        /// 初始化，返回是否成功
        bool Init();

        /// create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();

        /// IMU data access
        typedef std::shared_ptr<IMUData> IMUDataPtr;
        const std::vector<IMUData> &GetIMUData() const { return imu_data_; }

        /// get camera by id
        Camera::Ptr GetCamera(int camera_id) const
        {
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;
        std::vector<Camera::Ptr> cameras_;
        // files-based image lists (when dataset provides csv lists)
        std::vector<std::string> image_left_paths_;
        std::vector<std::string> image_right_paths_;
        std::vector<double> image_left_timestamps_;
        std::vector<double> image_right_timestamps_;

        // IMU data (if available)
        std::vector<IMUData> imu_data_;
    };
}

#endif // __DATASET_HPP__
