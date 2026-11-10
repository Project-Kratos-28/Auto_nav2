# Architecture

How the stack fits together. Setup and running: [`README.md`](../README.md).

```
Livox MID-360 ─► livox_ros_driver2 ─► /livox/lidar ─┬─► lidar_angle_filter ─► /livox/lidar_filtered ─► GLIM
                                   └► /livox/imu ───┼──────────────────────────────────────────────────► GLIM
                                                    └─► Nav2 obstacle_layer (local + global costmap)
GLIM (+ waypoint_manager) ─► TF map→odom→base_link, /glim_ros/odom, /glim_ros/map, waypoint services
/glim_ros/map ─► pcd2pgm (live) ─► /map ─► Nav2 global costmap
ZED 2i ─► ZED NEURAL depth (or ESS) ─► nvblox (odom frame) ─► /nvblox_node/static_map_slice ─► Nav2 local costmap
waypoint_mission.py ─► Nav2 navigate_to_pose / navigate_through_poses ─► /cmd_vel
```

- **GLIM** is the SLAM: pose (`map → odom → base_link`) and a 3D map, from the MID-360's points
  and IMU. Docker uses CPU odometry; a bare metal CUDA build can use GPU odometry with
  `glim_gpu:=true`.
- **pcd2pgm** turns GLIM's 3D map into a 2D `/map` (fixed 50×50 m grid, 5 cm) for the global costmap.
- **nvblox** covers what the LiDAR can't see: the MID-360's field of view is −7° to +52°, so at
  0.60 m height its lowest beam reaches the ground ~4.9 m out, and anything shorter than
  `0.60 − d·tan 7°` at distance `d` (a 30 cm rock at 2 m) is invisible to it. The ZED, tilted
  down, is fused by nvblox into a 3 cm map in GLIM's `odom` frame, whose 2D slice feeds the local
  costmap.
- **waypoint_manager** (a GLIM extension) tags waypoints relative to GLIM submaps, so they move
  with loop-closure corrections. **waypoint_mission.py** drives to them through Nav2.

The same `kratos.launch.py` runs in the `kratos_glim` Docker container by default, or on a
ROS 2 Jazzy host with `start.sh --bare-metal`. The host and Docker builds have separate
install directories; see `README.md` for setup and stop behavior.

## What starts

| Order | Component | Output |
|---|---|---|
| 1 | `livox_ros_driver2` | `/livox/lidar` (PointCloud2, 10 Hz), `/livox/imu` (200 Hz) |
| 2 | `lidar_angle_filter` | `/livox/lidar_filtered`: ±15° front and back removed (antenna); scans of a covered LiDAR dropped |
| 3 | `kratos_nav/nav.launch.py` | static TF `base_link → livox_frame`, `base_link → zed_camera_link`; Nav2 |
| 4 | GLIM (3 s later, after the static TF) | TF `map → odom → base_link`, `/glim_ros/odom`, `/glim_ros/map`, waypoint services |
| 5 | `pcd2pgm` (live) | `/map` |
| 6 | `kratos_perception` | ZED 2i, NEURAL or ESS depth, nvblox → `/nvblox_node/static_map_slice` |
| 7 | Cloudini (`cloudini:=true`) | `/glim_ros/points/compressed`, `/glim_ros/map/compressed` |
| 7 | RViz (`kratos_bringup/rviz/kratos.rviz`) | only with a GUI |

Nav2 is 7 nodes (controller, smoother, planner, behaviors, bt_navigator, waypoint_follower,
velocity_smoother) started directly: Jazzy's `navigation_launch.py` also starts route, collision
monitor and docking servers that abort the bringup without their own config. Controller and
behaviors publish `cmd_vel_nav`; the velocity smoother publishes the final `/cmd_vel`.

Measured on the Orin with both sensors: GLIM odometry 10 Hz, its TF ~120 ms behind real time;
ZED 15 Hz (NEURAL depth rate not measured yet); ESS ~40 ms/frame with TTA (processes ~11–15 Hz), ~20 ms without (27–28 Hz); nvblox
slice ~9–12 Hz; CPU 30–60 % per core, GPU ~37 %, RAM 11 GB of 62 GB.

## Known limits

- **Ramps:** nvblox (and the LiDAR layers) mark every surface 0.2–1.8 m above the *start* ground as
  an obstacle. A 10° ramp becomes a wall ~1.1 m up it (5°: 2.3 m); on a hill, ground can enter the band.
- **Ditches and drops are not detected** by any layer, and the planner allows unknown space
  (`allow_unknown: true`).
- **No relocalization:** waypoints and the map live only in the running GLIM session.
- **GLIM aborts on full LiDAR occlusion** (e.g. a hand over the sensor).
- **Fixed 50×50 m grid** around GLIM's start: size it to the arena.
- **ESS (`depth:=ess`) with TTA skips ~¼ of the camera's frames**; enough for the rover's speed.
- **Placeholders:** `lidar_z`, camera mount, grid size, lattice arc radius, speeds.
- **Not tested on the moving rover yet.** Tested on the Orin: all of the above bring-up with both
  sensors (bench), and the hardware-free tests.

## Repository layout

| Path | Contents |
|---|---|
| `start.sh`, `stop.sh`, `logs.sh`, `ros_network.env` | run the stack from the host |
| `docker/` | image (`Dockerfile.glim`, `build_image.sh`), container (`run_container.sh`), workspace build (`build_ws.sh`) |
| `tools/` | `check_time_sync.py`, `zed_hid_rebind.sh`, `setup_ptp.sh` |
| `src/kratos_bringup/` | `kratos.launch.py` (whole stack), RViz layouts (`kratos.rviz`, `laptop.rviz`) |
| `src/kratos_perception/` | ZED 2i + NEURAL/ESS depth + nvblox launch and config |
| `src/kratos_nav/` | Nav2 launch, params, behavior trees, `waypoint_mission.py`, tests |
| `src/pcd2pgm/` | point cloud → OccupancyGrid; live mode on a fixed grid |
| `src/lidar_angle_filter/` | front/back sector mask |
| `src/livox_ros_driver2/` | Livox driver (MID-360 only) |
| `glim/glim_config/` | GLIM configuration (keep the directory together) |
| `glim/glim_ext_addon/` | `waypoint_manager`, `waypoint_interfaces`, `glim_dump_export` |
| `glim/glim_ros_fix/` | patch for GLIM 1.2.2's `/glim_ros/map` bug (built into the image) |
| `docs/FIELD_TEST.md` | test-day runbook |
| `AGENTS.md` | orientation for AI agents and reviewers |
