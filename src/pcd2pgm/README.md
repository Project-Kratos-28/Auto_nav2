# pcd2pgm (Kratos live-mode fork)

Turns a 3D point cloud into a 2D `nav_msgs/OccupancyGrid` for Nav2.

Based on [LihanChen2004/pcd2pgm](https://github.com/LihanChen2004/pcd2pgm)
(Apache-2.0, see `LICENSE`). Kratos added a **live mode** that follows GLIM's
growing map. The original file mode still works.

| Mode | Turned on by | Input | Output |
|---|---|---|---|
| **Live** (what the rover uses) | `live_topic` is non-empty | `/glim_ros/map` (`PointCloud2`), about every 10 s | `/map` on a **fixed** grid, re-published on every input |
| File (upstream behaviour) | `live_topic` is empty | `pcd_file` (a `.pcd`), loaded once | `/map` sized to the cloud, re-published every 1 s |

There is **no command-line converter** (`ros2 run pcd2pgm pcd2pgm <in> <out>`
does not exist). The only executable is the `pcd2pgm_node` ROS node. To get a
`.pgm`/`.yaml` file, run the node and save `/map` with
`ros2 run nav2_map_server map_saver_cli -f <name>`.

## Pipeline (both modes)

1. Z pass-through: keep points with `thre_z_min <= z <= thre_z_max`
   (`flag_pass_through: false` keeps the band; `true` keeps everything *outside* it).
2. Radius outlier: drop points with fewer than `thres_point_count` neighbours
   within `thre_radius`.
3. Rasterize: every surviving point marks its cell 100 (occupied). All other
   cells are 0 (free). There are no -1 (unknown) cells.
4. Publish `/map` with QoS reliable + transient_local + keep_last(1), so a
   late subscriber (a restarted Nav2) still gets the last grid.
   The filtered cloud also goes out on `pcd_cloud` for RViz.

Live-mode specifics:

- **Fixed grid.** Origin and size come from parameters and never change, so
  Nav2's static layer updates in place and never resizes. Points outside the
  grid are dropped with a WARN (`... points fell outside the fixed grid bounds`).
- **Empty-cloud guard.** If nothing survives the filters, the node logs
  `... keeping the previous /map` and publishes nothing. It never publishes an
  all-free grid.
- **Full re-rasterize on each message.** GLIM's map is the whole
  loop-closure-corrected cloud, so nothing is lost between updates. The cost
  grows with the map. Each update logs `Live map update took N ms` (a real
  577k-point map took about 110 ms).
- `odom_to_lidar_odom` is ignored. GLIM's cloud is already in `map`.

## Run (live, as on the rover)

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  --params-file "$(ros2 pkg prefix pcd2pgm)/share/pcd2pgm/config/pcd2pgm_live.yaml"
```

GLIM only publishes `/glim_ros/map` while someone subscribes, and only every
~10 s. The first `/map` can therefore take up to 10 s after this starts.
The full stack is started by `start.sh` (see the top-level `README.md`); test-day checks are in `docs/FIELD_TEST.md`.

`ros2 launch pcd2pgm pcd2pgm_launch.py` is the upstream launch file. It uses
**file mode** (`config/pcd2pgm.yaml`) and opens RViz. Pass
`params_file:=<.../pcd2pgm_live.yaml>` to use it in live mode.

## Parameters

| Parameter | Default in code | `pcd2pgm_live.yaml` | Meaning |
|---|---|---|---|
| `live_topic` | `""` | `/glim_ros/map` | Non-empty turns on live mode |
| `fixed_origin_x`, `fixed_origin_y` | -25.0 | -25.0 | Grid corner in `map` (m) |
| `fixed_width_m`, `fixed_height_m` | 50.0 | 50.0 | Grid size (m). Must cover the **whole** arena |
| `map_resolution` | 0.05 | 0.05 | m per cell |
| `map_topic_name` | `map` | `map` | Output topic |
| `thre_z_min`, `thre_z_max` | 0.5, 2.0 | -0.40, 1.20 | Height band in GLIM's `map` frame (z=0 is the **LiDAR's start height**, not the ground) |
| `flag_pass_through` | false | false | false = keep the band |
| `thre_radius`, `thres_point_count` | 0.5, 10 | 0.75, 2 | Outlier filter. GLIM's map is voxelized at 0.5 m, so tighter values erase every wall (see the yaml comment) |
| `pcd_file` | `""` | unset | File mode only |
| `odom_to_lidar_odom` | zeros | unset | File mode only: [x,y,z,r,p,y] applied to the file cloud |

**Must match other files.** The height band is `0.2 m .. 1.8 m above the
ground` minus `lidar_z` (0.60 placeholder). If you change `lidar_z`, update
all three places:

- this yaml
- `min/max_obstacle_height` in `src/kratos_nav/config/nav2_params.yaml`
- the `lidar_z` argument of `src/kratos_nav/launch/nav.launch.py`

## Tests

`test/` holds a standalone live-mode suite (fixed grid, cell placement, height
band, outlier filter, out-of-bounds, empty-cloud guard, QoS, and timing on a
500k-point cloud). See `test/README.md`.

```bash
PCD2PGM_SETUP=$PWD/install/setup.bash bash src/pcd2pgm/test/run_all.sh
```

`colcon test` only runs the upstream linters (clang-format, clang-tidy, etc.),
not this suite.
