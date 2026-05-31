# 阅读[text](AGENTS.md)项目结构
> 阅读当前任务和已完成任务
> 理解当前任务完成任务重述部分
> 当接受任务重述时按照任务重述完成任务


# 当前任务
- 我需要进行运行数据的保存和回放，需要做到以下功能
1. 二进制地图文件（.bin）：保存关键帧 + 3D 地图点 + 位姿（极小、超快读写）
2. 纯文本轨迹文件（.txt）：方便调试、展示、离线回放
3. 格式需要适配evo
- 在本文件中设计bin文件和txt文件的格式
- 程序启动时接受一个参数决定是否保存数据

## 设计：二进制文件 `map.bin` 格式（紧凑）

- Header (fixed): 5 bytes magic = "VOMAP"
- Version: uint8_t (1)
- num_keyframes: uint64_t (little-endian)
- num_mappoints: uint64_t

- KeyFrame entries (repeated num_keyframes times):
	- id: uint64_t
	- timestamp: double (seconds)
	- tx, ty, tz: double x3 (translation)
	- qx, qy, qz, qw: double x4 (quaternion)

- MapPoint entries (repeated num_mappoints times):
	- id: uint64_t
	- x, y, z: double x3 (position)

Notes:
- All numeric values are written in native little-endian binary for speed.
- This format is minimal and designed for very fast binary IO and small overhead.

## 设计：纯文本轨迹 `trajectory.txt`（TUM / evo 兼容）

- Each line: timestamp tx ty tz qx qy qz qw
	- timestamp: seconds (floating point)
	- tx ty tz: translation (meters)
	- qx qy qz qw: orientation as quaternion

- This is the standard TUM trajectory format supported by `evo` for evaluation and visualization.

## 其他导出
- `mappoints.txt`: one `x y z` per line for 3D point cloud export.

## 运行时参数
- `--save_output=<dir>`: when provided, save `map.bin`, `trajectory.txt`, and `mappoints.txt` into `<dir>` after the run.

## 已实现（说明）
- 新增 `include/save/recorder.hpp` 和 `src/save/recorder.cpp`，实现 `Recorder::SaveAll(Map::Ptr)`，同时导出 `map.bin` 与 `trajectory.txt`、`mappoints.txt`。
- 在 `app/vo_app.cpp` 中添加 `--save_output` 参数支持；在程序结束时若提供该参数则调用保存。

## 验证
- 构建并运行示例（未自动运行测试）：
	- `cmake -S . -B build && cmake --build build`（已有构建脚本）
	- 运行： `./build/bin/vo_app --dataset_dir=/path/to/dataset --save_output=outdir`
	- 使用 `evo` 验证轨迹： `evo_traj tum outdir/trajectory.txt --ref ...` 或可视化。

# 已完成任务
## 
- 修改CMAKELISTS.txt增加一个宏选择是否开启Pangolin显示功能
- 当关闭时Pangolin部分不进行编译
##
- 将[text](include/interface/dataset.hpp)进行修改
- 让Dataset类的关键逻辑能够被重写，从而方便在拓展对其他数据集的适配时不需要修改更多的逻辑
