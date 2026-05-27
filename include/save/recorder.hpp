#pragma once
#ifndef __RECORDER_HPP__
#define __RECORDER_HPP__

#include "stereo_vo/map.hpp"

namespace my_slam {

// Recorder: save and load map / trajectory
class Recorder {
public:
    Recorder() = default;
    explicit Recorder(const std::string &out_dir) : out_dir_(out_dir) {}

    void SetOutputDir(const std::string &out_dir) { out_dir_ = out_dir; }

    // Save all keyframes and map points to binary and text files
    bool SaveAll(const Map::Ptr &map);

    // Load from binary (not implemented yet)
    bool LoadBin(const std::string &bin_path, Map::Ptr &map) { (void)bin_path; (void)map; return false; }

private:
    std::string out_dir_ = ".";

    bool SaveBin(const Map::Ptr &map);
    bool SaveTxt(const Map::Ptr &map);
};

} // namespace my_slam

#endif // __RECORDER_HPP__
