#include "interface/dataset.hpp"

#include <boost/format.hpp>
#include <fstream>
#include <opencv4/opencv2/opencv.hpp>
#include <filesystem>
#include <sstream>
#include <unordered_map>
using namespace std;

namespace my_slam
{
    bool Dataset::Init()
    {
        namespace fs = std::filesystem;
        fs::path data_root = fs::path(dataset_path_);
        if (fs::exists(data_root / "mav0"))
        {
            data_root /= "mav0";
        }
        else if (!fs::exists(data_root / "cam0") && fs::exists(data_root) && fs::is_directory(data_root))
        {
            for (const auto &entry : fs::directory_iterator(data_root))
            {
                if (!entry.is_directory())
                    continue;
                fs::path candidate = entry.path();
                if (fs::exists(candidate / "mav0"))
                {
                    data_root = candidate / "mav0";
                    break;
                }
                if (fs::exists(candidate / "cam0") && fs::exists(candidate / "imu0"))
                {
                    data_root = candidate;
                    break;
                }
            }
        }

        spdlog::info("Resolved dataset root: {}", data_root.string());

        auto load_sensor_yaml = [](const fs::path &yaml_path, double &fx, double &fy, double &cx, double &cy, SE3 &T_bs) -> bool {
            std::ifstream fin(yaml_path);
            if (!fin)
                return false;

            std::string line;
            bool in_intrinsics = false;
            bool in_tbs_data = false;
            std::vector<double> tbs_values;

            while (std::getline(fin, line))
            {
                auto pos = line.find('#');
                if (pos != std::string::npos)
                    line = line.substr(0, pos);
                if (line.find("intrinsics:") != std::string::npos)
                {
                    auto lb = line.find('[');
                    auto rb = line.find(']');
                    if (lb != std::string::npos && rb != std::string::npos && rb > lb)
                    {
                        std::stringstream ss(line.substr(lb + 1, rb - lb - 1));
                        std::string token;
                        std::vector<double> vals;
                        while (std::getline(ss, token, ','))
                        {
                            if (token.find_first_not_of(" \t\r\n") == std::string::npos)
                                continue;
                            vals.push_back(std::stod(token));
                        }
                        if (vals.size() >= 4)
                        {
                            fx = vals[0];
                            fy = vals[1];
                            cx = vals[2];
                            cy = vals[3];
                        }
                    }
                    continue;
                }

                if (line.find("T_BS:") != std::string::npos)
                {
                    in_tbs_data = false;
                    continue;
                }

                if (line.find("data:") != std::string::npos && line.find("T_BS") == std::string::npos)
                {
                    continue;
                }

                if (line.find("T_BS") != std::string::npos)
                {
                    in_tbs_data = false;
                    continue;
                }

                if (line.find("data:") != std::string::npos)
                {
                    in_tbs_data = true;
                    auto lb = line.find('[');
                    if (lb != std::string::npos)
                    {
                        std::string nums = line.substr(lb + 1);
                        if (!nums.empty() && nums.back() == ']')
                            nums.pop_back();
                        std::stringstream ss(nums);
                        std::string token;
                        while (std::getline(ss, token, ','))
                        {
                            if (token.find_first_not_of(" \t\r\n") == std::string::npos)
                                continue;
                            tbs_values.push_back(std::stod(token));
                        }
                    }
                    continue;
                }

                if (in_tbs_data)
                {
                    auto lb = line.find('[');
                    auto rb = line.find(']');
                    std::string nums = line;
                    if (lb != std::string::npos)
                        nums = line.substr(lb + 1);
                    if (rb != std::string::npos)
                        nums = nums.substr(0, nums.find(']'));
                    std::stringstream ss(nums);
                    std::string token;
                    while (std::getline(ss, token, ','))
                    {
                        if (token.find_first_not_of(" \t\r\n") == std::string::npos)
                            continue;
                        tbs_values.push_back(std::stod(token));
                    }
                }
            }

            if (tbs_values.size() >= 16)
            {
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 4; ++i)
                    for (int j = 0; j < 4; ++j)
                        T(i, j) = tbs_values[i * 4 + j];
                T_bs = SE3(T.block<3, 3>(0, 0), T.block<3, 1>(0, 3));
            }
            return fx != 0.0 && fy != 0.0;
        };

        // Prefer camera/imu CSV layout (body-style / EuRoC datasets)
        fs::path cam0 = data_root / "cam0";
        if (fs::exists(cam0))
        {
            // parse cam0 and cam1 data.csv with timestamps
            auto parse_cam = [&](const std::string &camname, std::vector<std::string> &out_paths, std::vector<double> &out_ts) {
                std::string csv = (data_root / camname / "data.csv").string();
                std::ifstream fin(csv);
                if (!fin)
                {
                    spdlog::warn("cannot open {}", csv);
                    return false;
                }
                std::string line;
                while (std::getline(fin, line))
                {
                    if (line.empty() || line[0] == '#')
                        continue;
                    std::string ts, fname;
                    if (line.find(',') != std::string::npos)
                    {
                        std::stringstream ss(line);
                        std::getline(ss, ts, ',');
                        std::getline(ss, fname, ',');
                    }
                    else
                    {
                        std::stringstream ss(line);
                        ss >> ts >> fname;
                    }
                    if (fname.empty())
                        fname = ts;
                    fs::path p = data_root / camname / "data" / fname;
                    out_paths.push_back(p.string());
                    try { out_ts.push_back(std::stod(ts)); } catch (...) { out_ts.push_back(0.0); }
                }
                return true;
            };

            parse_cam("cam0", image_left_paths_, image_left_timestamps_);
            parse_cam("cam1", image_right_paths_, image_right_timestamps_);

            double fx0 = 0, fy0 = 0, cx0 = 0, cy0 = 0;
            double fx1 = 0, fy1 = 0, cx1 = 0, cy1 = 0;
            SE3 T_bs0, T_bs1;
            bool cam0_ok = load_sensor_yaml(data_root / "cam0" / "sensor.yaml", fx0, fy0, cx0, cy0, T_bs0);
            bool cam1_ok = load_sensor_yaml(data_root / "cam1" / "sensor.yaml", fx1, fy1, cx1, cy1, T_bs1);
            if (cam0_ok && cam1_ok)
            {
                cameras_.clear();
                cameras_.push_back(std::make_shared<Camera>(fx0, fy0, cx0, cy0, (T_bs1.translation() - T_bs0.translation()).norm(), T_bs0));
                cameras_.push_back(std::make_shared<Camera>(fx1, fy1, cx1, cy1, (T_bs1.translation() - T_bs0.translation()).norm(), T_bs1));
            }

            // parse imu
            std::string imu_csv = (data_root / "imu0" / "data.csv").string();
            std::ifstream fim(imu_csv);
            if (fim)
            {
                std::string line;
                while (std::getline(fim, line))
                {
                    if (line.empty() || line[0] == '#')
                        continue;
                    std::stringstream ss(line);
                    if (line.find(',') != std::string::npos)
                    {
                        std::string ts, s1, s2, s3, s4, s5, s6;
                        std::getline(ss, ts, ',');
                        std::getline(ss, s1, ',');
                        std::getline(ss, s2, ',');
                        std::getline(ss, s3, ',');
                        std::getline(ss, s4, ',');
                        std::getline(ss, s5, ',');
                        std::getline(ss, s6, ',');
                        IMUData d;
                        try
                        {
                            d.timestamp = std::stod(ts);
                            d.gyro = Vec3(std::stod(s1), std::stod(s2), std::stod(s3));
                            d.acc = Vec3(std::stod(s4), std::stod(s5), std::stod(s6));
                        }
                        catch (...) { continue; }
                        imu_data_.push_back(d);
                    }
                    else
                    {
                        double t, gx, gy, gz, ax, ay, az;
                        ss >> t >> gx >> gy >> gz >> ax >> ay >> az;
                        IMUData d;
                        d.timestamp = t;
                        d.gyro = Vec3(gx, gy, gz);
                        d.acc = Vec3(ax, ay, az);
                        imu_data_.push_back(d);
                    }
                }
            }

            current_image_index_ = 0;
            spdlog::info("Dataset initialized from body-style CSVs: {} frames, {} imu samples", image_left_paths_.size(), imu_data_.size());
            return true;
        }
        ifstream fin(dataset_path_ + "/calib.txt");
        if (!fin)
        {
            spdlog::error("cannot find {}", dataset_path_ + "/calib.txt");
            return false;
        }

        for (int i = 0; i < 4; ++i)
        {
            char camera_name[3];
            for (int k = 0; k < 3; ++k)
            {
                fin >> camera_name[k];
            }
            double projection_data[12];
            for (int k = 0; k < 12; ++k)
            {
                fin >> projection_data[k];
            }
            Mat33 K;
            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            Vec3 t;
            t << projection_data[3], projection_data[7], projection_data[11];
            t = K.inverse() * t;
            K = K * 0.5;
            Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                              t.norm(), SE3(SO3(), t)));
            cameras_.push_back(new_camera);
        }
        fin.close();
        current_image_index_ = 0;
        return true;
    }
    
    Frame::Ptr Dataset::NextFrame()
    {
        cv::Mat image_left, image_right;
        double ts = 0.0;
        if (!image_left_paths_.empty() && !image_right_paths_.empty())
        {
            if (current_image_index_ >= (int)image_left_paths_.size() ||
                current_image_index_ >= (int)image_right_paths_.size())
            {
                spdlog::warn("no more images at index {}", current_image_index_);
                return nullptr;
            }
            std::string left_path = image_left_paths_[current_image_index_];
            std::string right_path = image_right_paths_[current_image_index_];
            image_left = cv::imread(left_path);
            image_right = cv::imread(right_path);

            if (image_left.empty() || image_right.empty())
            {
                spdlog::warn("imread returned empty Mat for index {}: left empty={} right empty={}",
                             current_image_index_, image_left.empty(), image_right.empty());
            }
            else
            {
                spdlog::info("Image left: rows={} cols={} type={} channels={}",
                              image_left.rows, image_left.cols, image_left.type(), image_left.channels());
                spdlog::info("Image right: rows={} cols={} type={} channels={}",
                              image_right.rows, image_right.cols, image_right.type(), image_right.channels());

                if (image_left.channels() == 3)
                    cv::cvtColor(image_left, image_left, cv::COLOR_BGR2GRAY);
                else if (image_left.channels() == 4)
                    cv::cvtColor(image_left, image_left, cv::COLOR_BGRA2GRAY);

                if (image_right.channels() == 3)
                    cv::cvtColor(image_right, image_right, cv::COLOR_BGR2GRAY);
                else if (image_right.channels() == 4)
                    cv::cvtColor(image_right, image_right, cv::COLOR_BGRA2GRAY);

                if (image_left.depth() != CV_8U)
                    image_left.convertTo(image_left, CV_8U);
                if (image_right.depth() != CV_8U)
                    image_right.convertTo(image_right, CV_8U);
            }
        }
        else
        {
            boost::format fmt("%s/image_%d/%06d.png");
            image_left =
                cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                           cv::IMREAD_GRAYSCALE);
            image_right =
                cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                           cv::IMREAD_GRAYSCALE);
        }

        if (image_left.empty() || image_right.empty())
        {
            spdlog::warn("cannot find images at index {}", current_image_index_);
            return nullptr;
        }

        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);

        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        new_frame->time_stamp_ = ts;
        current_image_index_++;
        return new_frame;
    }
}