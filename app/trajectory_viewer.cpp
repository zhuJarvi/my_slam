#include "common.hpp"
#include "arg_parser.hpp"

#ifdef USE_PANGOLIN
#include "pangolin/pangolin.h"
#endif

#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>

namespace
{
struct TrajectorySample
{
    double timestamp = 0.0;
    SE3 pose = SE3(); // stored as Tcw in the save file
};

bool LoadTrajectory(const std::string &path, std::vector<TrajectorySample> &trajectory)
{
    std::ifstream ifs(path);
    if (!ifs)
    {
        return false;
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        double timestamp = 0.0;
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
        {
            continue;
        }

        Eigen::Quaterniond q(qw, qx, qy, qz);
        if (q.norm() == 0.0)
        {
            continue;
        }

        q.normalize();

        Eigen::Vector3d t(tx, ty, tz);
        trajectory.push_back({timestamp, SE3(SO3(q), t)});
    }

    return !trajectory.empty();
}

bool LoadPoints(const std::string &path, std::vector<Eigen::Vector3d> &points)
{
    std::ifstream ifs(path);
    if (!ifs)
    {
        return false;
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        double x = 0.0, y = 0.0, z = 0.0;
        if (!(iss >> x >> y >> z))
        {
            continue;
        }

        points.emplace_back(x, y, z);
    }

    return !points.empty();
}

void DrawPose(const SE3 &pose, const float *color)
{
    const float size = 0.12f;
    const float fx = 400.0f;
    const float fy = 400.0f;
    const float cx = 512.0f;
    const float cy = 384.0f;
    const float width = 1080.0f;
    const float height = 768.0f;

    Sophus::Matrix4f m = pose.inverse().matrix().template cast<float>();
    glPushMatrix();
    glMultMatrixf(reinterpret_cast<const GLfloat *>(m.data()));

    if (color == nullptr)
    {
        glColor3f(1.0f, 0.0f, 0.0f);
    }
    else
    {
        glColor3f(color[0], color[1], color[2]);
    }

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(size * (0 - cx) / fx, size * (0 - cy) / fy, size);
    glVertex3f(0, 0, 0);
    glVertex3f(size * (0 - cx) / fx, size * (height - 1 - cy) / fy, size);
    glVertex3f(0, 0, 0);
    glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy, size);
    glVertex3f(0, 0, 0);
    glVertex3f(size * (width - 1 - cx) / fx, size * (0 - cy) / fy, size);

    glVertex3f(size * (width - 1 - cx) / fx, size * (0 - cy) / fy, size);
    glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy, size);

    glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy, size);
    glVertex3f(size * (0 - cx) / fx, size * (height - 1 - cy) / fy, size);

    glVertex3f(size * (0 - cx) / fx, size * (height - 1 - cy) / fy, size);
    glVertex3f(size * (0 - cx) / fx, size * (0 - cy) / fy, size);

    glVertex3f(size * (0 - cx) / fx, size * (0 - cy) / fy, size);
    glVertex3f(size * (width - 1 - cx) / fx, size * (0 - cy) / fy, size);
    glEnd();

    glPopMatrix();
}

void DrawTrajectory(const std::vector<TrajectorySample> &trajectory)
{
    const float red[3] = {1.0f, 0.0f, 0.0f};
    if (trajectory.empty())
    {
        return;
    }

    glLineWidth(2.0f);
    glColor3f(red[0], red[1], red[2]);
    glBegin(GL_LINE_STRIP);
    for (const auto &sample : trajectory)
    {
        const Eigen::Vector3d camera_center = sample.pose.inverse().translation();
        glVertex3d(camera_center.x(), camera_center.y(), camera_center.z());
    }
    glEnd();

    for (const auto &sample : trajectory)
    {
        DrawPose(sample.pose, red);
    }
}

void DrawPoints(const std::vector<Eigen::Vector3d> &points)
{
    if (points.empty())
    {
        return;
    }

    glPointSize(2.0f);
    glColor3f(0.0f, 0.6f, 0.1f);
    glBegin(GL_POINTS);
    for (const auto &point : points)
    {
        glVertex3d(point.x(), point.y(), point.z());
    }
    glEnd();
}
} // namespace

int main(int argc, char **argv)
{
    ArgParser ap(argc, argv);

    std::string trajectory_file;
    std::string points_file;
    std::string window_name;

    ap.get<std::string>("trajectory_file", trajectory_file, "trajectory.txt", ArgParser::to_string);
    ap.get<std::string>("mappoints_file", points_file, "mappoints.txt", ArgParser::to_string);
    ap.get<std::string>("window_name", window_name, "Trajectory Viewer", ArgParser::to_string);

    if (points_file == "mappoints.txt")
    {
        for (int i = 1; i < argc; ++i)
        {
            std::string arg(argv[i]);
            const std::string typo_prefix = "--mappints_file=";
            if (arg.rfind(typo_prefix, 0) == 0)
            {
                points_file = arg.substr(typo_prefix.size());
                break;
            }
        }
    }

    std::vector<TrajectorySample> trajectory;
    std::vector<Eigen::Vector3d> points;

    if (!LoadTrajectory(trajectory_file, trajectory))
    {
        spdlog::error("Failed to load trajectory file: {}", trajectory_file);
        return -1;
    }

    if (std::filesystem::exists(points_file))
    {
        if (!LoadPoints(points_file, points))
        {
            spdlog::warn("Failed to load point file or file is empty: {}", points_file);
        }
    }
    else
    {
        spdlog::warn("Point file does not exist: {}", points_file);
    }

    spdlog::info("Loaded {} trajectory samples from {}", trajectory.size(), trajectory_file);
    spdlog::info("Loaded {} map points from {}", points.size(), points_file);

#ifdef USE_PANGOLIN
    pangolin::CreateWindowAndBind(window_name, 1280, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1280, 768, 500, 500, 640, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -8, -12, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &display = pangolin::CreateDisplay()
                                   .SetBounds(0.0, 1.0, 0.0, 1.0, -1280.0f / 768.0f)
                                   .SetHandler(new pangolin::Handler3D(vis_camera));

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        display.Activate(vis_camera);
        DrawPoints(points);
        DrawTrajectory(trajectory);

        pangolin::FinishFrame();
        usleep(5000);
    }
#else
    spdlog::error("This target requires USE_PANGOLIN=ON at CMake configure time.");
    return -1;
#endif

    return 0;
}