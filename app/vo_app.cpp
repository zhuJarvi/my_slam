#include "common.hpp"
#include "stereo_vo/frontend.hpp"
#include "stereo_vo/map.hpp"
#include "stereo_vo/backend.hpp"
#include "interface/viewer.hpp"
#include "interface/dataset.hpp"
#include "arg_parser.hpp"
#include "opencv4/opencv2/opencv.hpp"
std::string dataset_dir;
int main(int argc, char **argv){
    ArgParser ap(argc, argv);
    ap.get<std::string>("dataset_dir", dataset_dir, "dataset/EuRoC/MH_01_easy/", ArgParser::to_string);
    spdlog::info("dataset_dir = {}", dataset_dir);
    my_slam::Dataset::Ptr dataset(new my_slam::Dataset(dataset_dir));

    if (!dataset->Init())
    {
        spdlog::error("Failed to initialize dataset.");
        return -1;
    }

    my_slam::Frontend::Ptr frontend(new my_slam::Frontend(ap));
    my_slam::Backend::Ptr backend(new my_slam::Backend());
    my_slam::Map::Ptr map(new my_slam::Map());
    my_slam::Viewer::Ptr viewer(new my_slam::Viewer());

    // prepare IMU stream index
    const auto &imu_all = dataset->GetIMUData();
    size_t imu_idx = 0;

    viewer->SetMap(map);

    frontend->SetMap(map);
    frontend->SetViewer(viewer);
    frontend->SetBackend(backend);
    frontend->SetCameras(dataset->GetCamera(0), dataset->GetCamera(1));

    backend->SetMap(map);
    backend->SetCameras(dataset->GetCamera(0), dataset->GetCamera(1));
    spdlog::info("VO app initialized.");


    while (true)
    {
        auto frame = dataset->NextFrame();
        if (!frame)
        {
            spdlog::info("No more frames. Exiting.");
            break;
        }
        // feed IMU data up to current frame timestamp
        while (imu_idx < imu_all.size() && imu_all[imu_idx].timestamp <= frame->time_stamp_)
        {
            frontend->AddIMU(imu_all[imu_idx]);
            imu_idx++;
        }
        if (!frontend->AddFrame(frame))
        {
            spdlog::error("Failed to add frame.");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    backend->Stop();
    viewer->Close();
}
