#include "common.hpp"
#include "stereo_vo/frontend.hpp"
#include "stereo_vo/map.hpp"
#include "stereo_vo/backend.hpp"
#include "interface/viewer.hpp"
#include "interface/dataset.hpp"
#include "save/recorder.hpp"
#include "arg_parser.hpp"

#include <chrono>
#include <filesystem>
#include <opencv4/opencv2/opencv.hpp>

namespace my_slam
{
namespace
{
std::string ResolveCalibrationFile(const std::string &requested_path)
{
    namespace fs = std::filesystem;
    if (requested_path.empty())
    {
        return {};
    }

    const fs::path candidate(requested_path);
    if (candidate.is_absolute() && fs::exists(candidate))
    {
        return candidate.string();
    }

    const fs::path cwd = fs::current_path();
    const std::vector<fs::path> search_paths = {
        candidate,
        cwd / candidate,
        cwd.parent_path() / candidate,
        cwd.parent_path().parent_path() / candidate,
    };

    for (const auto &path : search_paths)
    {
        if (fs::exists(path))
        {
            return path.string();
        }
    }

    return candidate.string();
}

SE3 ReadStereoPose(const cv::Mat &rotation, const cv::Mat &translation)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            R(row, col) = rotation.at<double>(row, col);
        }
    }

    Vec3 t(translation.at<double>(0, 0), translation.at<double>(1, 0), translation.at<double>(2, 0));
    return SE3(SO3(R), t);
}

} // namespace

class LiveStereoDataset : public Dataset
{
public:
    LiveStereoDataset(const std::string &calib_file,
                      int id,
                      int width,
                      int height,
                      int fps,
                      double fx,
                      double fy,
                      double cx,
                      double cy,
                      double baseline)
        : Dataset(calib_file), calib_file_(calib_file), id_(id), width_(width), height_(height), fps_(fps), fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline)
    {
    }

    bool Init() override
    {
        const std::string calib_path = ResolveCalibrationFile(calib_file_);
        const int requested_capture_width = width_;
        const int requested_capture_height = height_;
        const int requested_capture_fps = fps_;

        SE3 right_pose(SO3(), Vec3(baseline_, 0, 0));
        bool rectification_ready = false;
        cv::Mat left_map1, left_map2, right_map1, right_map2;

        if (!calib_path.empty())
        {
            cv::FileStorage fs(calib_path, cv::FileStorage::READ);
            if (fs.isOpened())
            {
                int yaml_width = 0;
                int yaml_height = 0;
                cv::Mat left_camera_matrix;
                cv::Mat right_camera_matrix;
                cv::Mat left_dist_coeffs;
                cv::Mat right_dist_coeffs;
                cv::Mat rect_R1, rect_R2;
                cv::Mat rect_P1, rect_P2;
                cv::Mat stereo_R;
                cv::Mat stereo_T;
                fs["image_width"] >> yaml_width;
                fs["image_height"] >> yaml_height;
                fs["left_camera_matrix"] >> left_camera_matrix;
                fs["right_camera_matrix"] >> right_camera_matrix;
                fs["left_dist_coeffs"] >> left_dist_coeffs;
                fs["right_dist_coeffs"] >> right_dist_coeffs;
                fs["R1"] >> rect_R1;
                fs["R2"] >> rect_R2;
                fs["P1"] >> rect_P1;
                fs["P2"] >> rect_P2;
                fs["R"] >> stereo_R;
                fs["T"] >> stereo_T;

                if (width_ <= 0 && yaml_width > 0)
                {
                    width_ = yaml_width;
                }
                if (height_ <= 0 && yaml_height > 0)
                {
                    height_ = yaml_height;
                }

                auto read_intrinsics = [](const cv::Mat &K, double &fx, double &fy, double &cx, double &cy) -> bool {
                    if (K.empty() || K.rows < 3 || K.cols < 3)
                    {
                        return false;
                    }
                    fx = K.at<double>(0, 0);
                    fy = K.at<double>(1, 1);
                    cx = K.at<double>(0, 2);
                    cy = K.at<double>(1, 2);
                    return fx > 0.0 && fy > 0.0 && cx > 0.0 && cy > 0.0;
                };

                if (!rect_P1.empty() && !rect_P2.empty())
                {
                    read_intrinsics(rect_P1, fx_, fy_, cx_, cy_);
                    if (rect_P2.rows >= 3 && rect_P2.cols >= 4 && rect_P2.at<double>(0, 0) != 0.0)
                    {
                        baseline_ = rect_P2.at<double>(0, 3) / rect_P2.at<double>(0, 0);
                    }
                    right_pose = SE3(SO3(), Vec3(baseline_, 0, 0));
                    if (!rect_R1.empty() && !rect_R2.empty())
                    {
                        if (!left_dist_coeffs.empty() && !right_dist_coeffs.empty() &&
                            width_ > 0 && height_ > 0)
                        {
                            cv::Size image_size(width_, height_);
                            cv::initUndistortRectifyMap(left_camera_matrix, left_dist_coeffs,
                                                        rect_R1, rect_P1, image_size,
                                                        CV_32FC1, left_map1, left_map2);
                            cv::initUndistortRectifyMap(right_camera_matrix, right_dist_coeffs,
                                                        rect_R2, rect_P2, image_size,
                                                        CV_32FC1, right_map1, right_map2);
                            rectification_ready = true;
                        }
                    }
                }
                else if (fx_ <= 0.0 || fy_ <= 0.0 || cx_ <= 0.0 || cy_ <= 0.0)
                {
                    if (!read_intrinsics(left_camera_matrix, fx_, fy_, cx_, cy_))
                    {
                        read_intrinsics(right_camera_matrix, fx_, fy_, cx_, cy_);
                    }
                }

                if (!rectification_ready && !stereo_R.empty() && !stereo_T.empty() &&
                    left_camera_matrix.data != nullptr && right_camera_matrix.data != nullptr &&
                    !left_dist_coeffs.empty() && !right_dist_coeffs.empty() && width_ > 0 && height_ > 0)
                {
                    cv::Mat R1, R2, P1, P2, Q;
                    cv::stereoRectify(left_camera_matrix, left_dist_coeffs,
                                      right_camera_matrix, right_dist_coeffs,
                                      cv::Size(width_, height_), stereo_R, stereo_T,
                                      R1, R2, P1, P2, Q);
                    read_intrinsics(P1, fx_, fy_, cx_, cy_);
                    if (P2.rows >= 3 && P2.cols >= 4 && P2.at<double>(0, 0) != 0.0)
                    {
                        baseline_ = P2.at<double>(0, 3) / P2.at<double>(0, 0);
                    }
                    right_pose = SE3(SO3(), Vec3(baseline_, 0, 0));
                    cv::Size image_size(width_, height_);
                    cv::initUndistortRectifyMap(left_camera_matrix, left_dist_coeffs,
                                                R1, P1, image_size,
                                                CV_32FC1, left_map1, left_map2);
                    cv::initUndistortRectifyMap(right_camera_matrix, right_dist_coeffs,
                                                R2, P2, image_size,
                                                CV_32FC1, right_map1, right_map2);
                    rectification_ready = true;
                }

                if (baseline_ <= 0.0 && !stereo_R.empty() && !stereo_T.empty() && stereo_T.rows >= 3 && stereo_T.cols >= 1)
                {
                    right_pose = ReadStereoPose(stereo_R, stereo_T);
                    baseline_ = std::sqrt(stereo_T.at<double>(0, 0) * stereo_T.at<double>(0, 0) +
                                          stereo_T.at<double>(1, 0) * stereo_T.at<double>(1, 0) +
                                          stereo_T.at<double>(2, 0) * stereo_T.at<double>(2, 0));
                }

                spdlog::info("Loaded stereo calibration from {}", calib_path);
            }
            else
            {
                spdlog::warn("Could not open calibration file '{}', falling back to CLI parameters.", calib_path);
            }
        }

        cap_.open(id_, cv::CAP_ANY);
        if (!cap_.isOpened())
        {
            spdlog::error("Failed to open stereo cameras: camera={}", id_);
            return false;
        }
        cap_.set(cv::CAP_PROP_BRIGHTNESS, 50);
        cap_.set(cv::CAP_PROP_CONTRAST, 50);
        cap_.set(cv::CAP_PROP_SATURATION, 50);
        cap_.set(cv::CAP_PROP_HUE, 0);
        cap_.set(cv::CAP_PROP_GAMMA, 100);
        cap_.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 50);
        cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
        cap_.set(cv::CAP_PROP_EXPOSURE, 10);
        cap_.set(cv::CAP_PROP_SHARPNESS, 50);
        cap_.set(cv::CAP_PROP_BACKLIGHT, 0);

        if (requested_capture_width > 0)
        {
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, requested_capture_width);
        }
        if (requested_capture_height > 0)
        {
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, requested_capture_height);
        }
        if (requested_capture_fps > 0)
        {
            cap_.set(cv::CAP_PROP_FPS, requested_capture_fps);
        }

        if (fx_ <= 0.0 || fy_ <= 0.0 || cx_ <= 0.0 || cy_ <= 0.0)
        {
            spdlog::error("Invalid stereo calibration parameters: fx fy cx cy must be positive");
            return false;
        }
        if (baseline_ <= 0.0)
        {
            baseline_ = 0.0663;
        }

        cameras_.clear();
        cameras_.push_back(std::make_shared<Camera>(fx_, fy_, cx_, cy_, baseline_, SE3(SO3(), Vec3::Zero())));
        cameras_.push_back(std::make_shared<Camera>(fx_, fy_, cx_, cy_, baseline_, right_pose));

        current_image_index_ = 0;
        start_time_ = std::chrono::steady_clock::now();
        rectification_ready_ = rectification_ready;
        left_rectify_map1_ = left_map1;
        left_rectify_map2_ = left_map2;
        right_rectify_map1_ = right_map1;
        right_rectify_map2_ = right_map2;
        spdlog::info("Live stereo dataset initialized: camera={}, {}x{}@{}, calib={}", id_, width_, height_, fps_, calib_path);
        return true;
    }

    Frame::Ptr NextFrame() override
    {
        cv::Mat src;

        if (!cap_.read(src))
        {
            spdlog::warn("Failed to read stereo frames.");
            return nullptr;
        }

        if (src.empty())
        {
            spdlog::warn("Captured empty stereo frame.");
            return nullptr;
        }

        cv::Mat src_gray;
        if (src.channels() == 3)
        {
            cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
        }
        else if (src.channels() == 4)
        {
            cv::cvtColor(src, src_gray, cv::COLOR_BGRA2GRAY);
        }
        else
        {
            src_gray = src;
        }

        if (src_gray.depth() != CV_8U)
        {
            src_gray.convertTo(src_gray, CV_8U);
        }

        auto new_frame = Frame::CreateFrame();
        const int half = src_gray.cols / 2;
        if (half <= 0)
        {
            spdlog::warn("Stereo frame width is too small: {}", src_gray.cols);
            return nullptr;
        }

        cv::Mat left_roi = src_gray(cv::Rect(0, 0, half, src_gray.rows)).clone();
        cv::Mat right_roi = src_gray(cv::Rect(half, 0, src_gray.cols - half, src_gray.rows)).clone();

        if (rectification_ready_)
        {
            cv::remap(left_roi, left_roi, left_rectify_map1_, left_rectify_map2_, cv::INTER_LINEAR);
            cv::remap(right_roi, right_roi, right_rectify_map1_, right_rectify_map2_, cv::INTER_LINEAR);
        }

        new_frame->left_img_ = left_roi;
        new_frame->right_img_ = right_roi;

        const auto now = std::chrono::steady_clock::now();
        new_frame->time_stamp_ = std::chrono::duration<double>(now - start_time_).count();
        current_image_index_++;
        return new_frame;
    }

    const std::vector<IMUData> &GetIMUData() const override
    {
        return empty_imu_data_;
    }

private:
    std::string calib_file_;
    int id_ = 0;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double baseline_ = 0.0;

    bool rectification_ready_ = false;
    cv::Mat left_rectify_map1_;
    cv::Mat left_rectify_map2_;
    cv::Mat right_rectify_map1_;
    cv::Mat right_rectify_map2_;

    cv::VideoCapture cap_;
    std::chrono::steady_clock::time_point start_time_;
    std::vector<IMUData> empty_imu_data_;
};
} // namespace my_slam

using namespace my_slam;

int main(int argc, char **argv)
{
    ArgParser ap(argc, argv);

    int camera_id = 0;
    int camera_width = 0;
    int camera_height = 0;
    int camera_fps = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double baseline = 0.0663;
    std::string calib_file = "stereo_calib_6x5_0195.yml";
    std::string save_output;

    ap.get<int>("camera_id", camera_id, 0, ArgParser::to_int);
    ap.get<int>("camera_width", camera_width, 0, ArgParser::to_int);
    ap.get<int>("camera_height", camera_height, 0, ArgParser::to_int);
    ap.get<int>("camera_fps", camera_fps, 0, ArgParser::to_int);
    ap.get<double>("fx", fx, 0.0, ArgParser::to_double);
    ap.get<double>("fy", fy, 0.0, ArgParser::to_double);
    ap.get<double>("cx", cx, 0.0, ArgParser::to_double);
    ap.get<double>("cy", cy, 0.0, ArgParser::to_double);
    ap.get<double>("baseline", baseline, 0.0663, ArgParser::to_double);
    ap.get<std::string>("calib_file", calib_file, calib_file, ArgParser::to_string);
    ap.get<std::string>("save_output", save_output, "", ArgParser::to_string);

    auto dataset = std::make_shared<LiveStereoDataset>(
        calib_file,
        camera_id,
        camera_width,
        camera_height,
        camera_fps,
        fx,
        fy,
        cx,
        cy,
        baseline);

    if (!dataset->Init())
    {
        return -1;
    }

    Frontend::Ptr frontend(new Frontend(ap));
    Backend::Ptr backend(new Backend());
    Map::Ptr map(new Map());
    Viewer::Ptr viewer(new Viewer());
    Recorder recorder;
    if (!save_output.empty())
    {
        recorder.SetOutputDir(save_output);
    }

    viewer->SetMap(map);

    frontend->SetMap(map);
    frontend->SetViewer(viewer);
    frontend->SetBackend(backend);
    frontend->SetCameras(dataset->GetCamera(0), dataset->GetCamera(1));

    backend->SetMap(map);
    backend->SetCameras(dataset->GetCamera(0), dataset->GetCamera(1));
    spdlog::info("Live stereo VO initialized.");

    for (int warmup = 0; warmup < 10; ++warmup)
    {
        if (!dataset->NextFrame())
        {
            break;
        }
    }

    while (true)
    {
        auto frame = dataset->NextFrame();
        if (!frame)
        {
            spdlog::warn("No more live frames. Exiting.");
            break;
        }

        if (!frontend->AddFrame(frame))
        {
            spdlog::error("Failed to add live stereo frame.");
            break;
        }
    }

    backend->Stop();
    viewer->Close();
    if (!save_output.empty())
    {
        spdlog::info("Saving live map to {}", save_output);
        recorder.SaveAll(map);
    }

    return 0;
}