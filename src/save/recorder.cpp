#include "save/recorder.hpp"
#include <fstream>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <iomanip>

namespace my_slam {

static void write_string(std::ofstream &ofs, const std::string &s) {
    ofs.write(s.data(), s.size());
}

bool Recorder::SaveAll(const Map::Ptr &map) {
    namespace fs = std::filesystem;
    if (!fs::exists(out_dir_)) {
        fs::create_directories(out_dir_);
    }
    bool a = SaveBin(map);
    bool b = SaveTxt(map);
    return a && b;
}

bool Recorder::SaveBin(const Map::Ptr &map) {
    namespace fs = std::filesystem;
    std::string path = (fs::path(out_dir_) / "map.bin").string();
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;

    // header: magic, version
    const char magic[5] = {'V','O','M','A','P'};
    ofs.write(magic, 5);
    uint8_t version = 1;
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));

    // collect keyframes and map points
    auto kfs = map->GetAllKeyFrames();
    auto mps = map->GetAllMapPoints();

    uint64_t num_kf = kfs.size();
    uint64_t num_mp = mps.size();
    ofs.write(reinterpret_cast<const char*>(&num_kf), sizeof(num_kf));
    ofs.write(reinterpret_cast<const char*>(&num_mp), sizeof(num_mp));

    // write keyframes: id, timestamp, tx,ty,tz, qx,qy,qz,qw (doubles)
    for (const auto &kv : kfs) {
        uint64_t id = kv.first;
        auto f = kv.second;
        double ts = f->time_stamp_;
        SE3 T = f->GetPose();
        Vec3 t = T.translation();
        Eigen::Quaterniond q(T.rotationMatrix());
        ofs.write(reinterpret_cast<const char*>(&id), sizeof(id));
        ofs.write(reinterpret_cast<const char*>(&ts), sizeof(ts));
        ofs.write(reinterpret_cast<const char*>(t.data()), sizeof(double)*3);
        double qv[4] = {q.x(), q.y(), q.z(), q.w()};
        ofs.write(reinterpret_cast<const char*>(qv), sizeof(double)*4);
    }

    // write map points: id, x,y,z (doubles)
    for (const auto &kv : mps) {
        uint64_t id = kv.first;
        auto mp = kv.second;
        Vec3 p = mp->Pos();
        ofs.write(reinterpret_cast<const char*>(&id), sizeof(id));
        ofs.write(reinterpret_cast<const char*>(p.data()), sizeof(double)*3);
    }

    ofs.close();
    return true;
}

bool Recorder::SaveTxt(const Map::Ptr &map) {
    namespace fs = std::filesystem;
    // trajectory tum format: timestamp tx ty tz qx qy qz qw
    std::string traj_path = (fs::path(out_dir_) / "trajectory.txt").string();
    std::ofstream tof(traj_path);
    if (!tof) return false;

    auto kfs = map->GetAllKeyFrames();
    // gather into vector and sort by timestamp
    std::vector<std::pair<double, Frame::Ptr>> vec;
    vec.reserve(kfs.size());
    for (auto &kv : kfs) {
        vec.emplace_back(kv.second->time_stamp_, kv.second);
    }
    std::sort(vec.begin(), vec.end(), [](auto &a, auto &b){ return a.first < b.first; });
    for (auto &p : vec) {
        auto f = p.second;
        SE3 T = f->GetPose();
        Vec3 t = T.translation();
        Eigen::Quaterniond q(T.rotationMatrix());
        // TUM: timestamp tx ty tz qx qy qz qw
        tof << std::fixed << std::setprecision(6) << p.first << " ";
        tof << t(0) << " " << t(1) << " " << t(2) << " ";
        tof << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    tof.close();

    // write map points as xyz
    std::string mp_path = (fs::path(out_dir_) / "mappoints.txt").string();
    std::ofstream mpof(mp_path);
    if (!mpof) return false;
    auto mps = map->GetAllMapPoints();
    for (auto &kv : mps) {
        Vec3 p = kv.second->Pos();
        mpof << std::fixed << std::setprecision(6) << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
    mpof.close();
    return true;
}

} // namespace my_slam
