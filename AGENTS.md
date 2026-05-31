# AGENTS.md

## Purpose

This repository implements a stereo visual-odometry pipeline in C++17 with CMake.
Use this file as the first-stop guide for safe, productive agent changes.

## Quick Start

Build from the repository root:

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

Run:

```bash
./build/bin/vo_app --dataset_dir <path-to-dataset>
```

Notes:
- Main executable output is `build/bin/vo_app`.
- There is no automated test target configured in CMake; validate changes by building and running `vo_app`.

## Project Layout

- `app/`: Application entrypoint (`vo_app.cpp`) and app target.
- `src/interface/`: IO/utility layer (`camera`, `dataset`, `viewer`).
- `src/stereo_vo/`: VO core (`frontend`, `backend`, `map`, and `struct_base/*`).
- `include/`: Public headers mirrored by module.
- `config/default.yaml`: Example runtime parameters and dataset path.
- `build/`: Generated artifacts (do not edit manually).

## Architecture Boundaries

- `vo_interface` (shared library): camera model, dataset loading, visualization.
- `vo_core` (shared library): tracking, mapping, optimization logic.
- `vo_app` wires dataset + frontend + backend + map + viewer and runs the frame loop.

When changing behavior:
- Keep data-structure ownership in `struct_base` (`Frame`, `Feature`, `MapPoint`).
- Keep dataset format assumptions inside `Dataset`.
- Keep tracking/keyframe policy inside `Frontend`.

## Repo-Specific Conventions

- Namespace is `my_slam`.
- Smart-pointer alias pattern is common: `using/typedef ... Ptr`.
- Member fields typically end with `_`.
- Logging uses `spdlog` instead of `std::cout`.
- Math and common type aliases are centralized in `include/common.hpp`.

## Dependencies and Assumptions

Required dependencies in CMake:
- OpenCV 4
- Eigen3
- Sophus
- g2o + CSparse
- Ceres
- Pangolin
- spdlog

`Dataset::Init()` expects KITTI-style files such as `calib.txt` and image folders (`image_0`, `image_1`) under the dataset directory.

## Agent Guardrails

- Prefer minimal, localized changes; avoid broad refactors unless requested.
- Do not edit generated files under `build/`.
- Update matching headers/sources together when changing interfaces under `include/` and `src/`.
- If adding new targets or files, update `CMakeLists.txt` in the relevant directory.

## Useful Entry Files

- `app/vo_app.cpp`
- `CMakeLists.txt`
- `include/common.hpp`
- `src/stereo_vo/frontend.cpp`
- `src/interface/dataset.cpp`