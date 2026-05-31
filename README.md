# stereo_vo

Stereo visual odometry pipeline in C++17 with CMake.

## Build

The project enables Pangolin by default through the top-level CMake option `USE_PANGOLIN`.

```bash
cmake -S . -B build
cmake --build build -j
```

If you want to skip Pangolin-based targets, configure with:

```bash
cmake -S . -B build -DUSE_PANGOLIN=OFF
```

## Run VO And Save Output

Run the main VO application and export the map / trajectory files:

```bash
./build/bin/vo_app --dataset_dir=/path/to/dataset --save_output=outdir
```

This writes the following files into `outdir`:

- `map.bin`
- `trajectory.txt`
- `mappoints.txt`

## Visualize Saved Trajectory And Map Points

When `USE_PANGOLIN=ON`, the build also generates a separate viewer target:

- `vo_pangolin_viewer`

Use it to visualize the exported trajectory and point cloud:

```bash
./build/bin/vo_pangolin_viewer --trajectory_file=outdir/trajectory.txt --mappoints_file=outdir/mappoints.txt
```

Optional parameter:

- `--window_name=Trajectory Viewer`

The trajectory file uses the TUM / evo format:

```text
timestamp tx ty tz qx qy qz qw
```

The point file uses one `x y z` sample per line.