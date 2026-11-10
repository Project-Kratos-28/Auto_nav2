# AGENTS.md

Guidance for AI coding agents and new teammates. Read [`README.md`](README.md) first (setup and running), then
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) and [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md).

## What this branch is

`jazzy-nvblox`: the Kratos rover autonomy stack on a Jetson AGX Orin (JetPack 7.2, Ubuntu 24.04,
ROS 2 Jazzy), in Docker by default (`kratos_glim`, built from `docker/`) or on the host with
`start.sh --bare-metal` and a separate `build_bare.sh` build:
Livox MID-360 → GLIM (SLAM + `waypoint_manager`) → pcd2pgm `/map` → Nav2 → `/cmd_vel`, plus
ZED 2i → ESS/ZED depth → nvblox → Nav2 local costmap. `start.sh` runs everything
(`src/kratos_bringup/launch/kratos.launch.py`) in both modes.

**Scope ends at `/cmd_vel`.** Whatever drives the wheels is outside this repo; don't add or
discuss anything downstream of it.

## Build and test

The Docker build and tests run in the container (`docker/run_container.sh <cmd>`, working directory
`/workspaces/kratos_glim`). For bare metal, build with `./build_bare.sh` and launch with
`./start.sh --bare-metal`:

```bash
docker/run_container.sh docker/build_ws.sh
docker/run_container.sh 'colcon test --packages-select waypoint_manager lidar_angle_filter && colcon test-result --all'
docker/run_container.sh python3 src/kratos_bringup/test/test_lidar_ip.py
docker/run_container.sh 'KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh'
docker/run_container.sh 'MISSION_ARGS="-p mode:=through" KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh'
docker/run_container.sh 'KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/mission_edge_test.sh'
docker/run_container.sh 'PCD2PGM_SETUP=$PWD/install/setup.bash bash src/pcd2pgm/test/run_all.sh'
```

- `build_ws.sh` lists the packages explicitly: add new packages there.
- The shell tests use `ROS_DOMAIN_ID` 43–45 and `pkill` their own node names: run them one at a
  time, never during a live run.
- The ZED and the GPU may be in use by another stack (`~/kratos_nvblox`, a separate project).
  Only one process can open the ZED; don't start `perception.launch.py` or `start.sh` with a
  camera while another container runs it.

## Invariants (breaking these fails silently in the field)

- **`lidar_z` in four places must match:** `start.sh lidar_z:=` (→ `nav.launch.py` static TF and
  `perception.launch.py`, which computes nvblox's slice band), `nav2_params.yaml`
  (`min/max_obstacle_height`, both costmaps), `pcd2pgm_live.yaml` (`thre_z_min/max`). Heights are
  relative to GLIM's `map` z=0, the LiDAR's start height; the ground is at −`lidar_z`.
- **TF chain `map → odom → base_link → {livox_frame, zed_camera_link}`.** GLIM publishes the first
  two links; both mount links are static, from `nav.launch.py`. The ZED must never track or publish
  TF (`kratos_perception/config/zed2i_glim.yaml`). Without `base_link → livox_frame`, GLIM
  publishes no pose TF.
- **nvblox maps in `odom`, and `nvblox_layer` is in the local costmap only** (GLIM's loop closures
  move `map → odom`).
- **GLIM runs without its OpenGL viewer when there is no display** (`kratos.launch.py` writes a
  config copy without `libstandard_viewer.so`); with the viewer and no display it crashes at start.
  `librviz_viewer.so` must stay: it publishes `/glim_ros/map`.
- **The patched `glim_ros` overlay** (`/opt/glim_ros_fix_ws`) is sourced last in the container;
  check with `ros2 pkg prefix glim_ros`. Without it `/glim_ros/map` gets phantom walls.
- **pcd2pgm live mode uses a fixed grid** (origin/size never change: Nav2's static layer must not
  resize). `/map` cells are only 0 or 100; an empty filtered cloud keeps the previous `/map`, never
  an all-free grid; QoS reliable + transient_local + depth 1. Radius filter stays loose
  (0.75 m / 2 neighbours): GLIM's map is voxelized at 0.5 m.
- **waypoint_manager:** pending waypoints are listed as `"<name> (pending)"`, return `found=False`,
  and are saved under `pending_waypoints`; consumers strip the suffix (`waypoint_mission.py`).
- **Nav2 (Jazzy):** plugin names use `::`; `nav.launch.py` starts the 7 nodes itself (Jazzy's
  `navigation_launch.py` adds servers that need their own config); both BT keys must exist in
  `nav2_params.yaml`, and the trees must not use Spin or BackUp (the rover must never reverse: MPPI
  `vx_min: 0.0`). Controller: RotationShim around MPPI; MPPI's `model_dt` must equal
  1 / `controller_frequency`. `lattice_filepath` is filled in by `nav.launch.py`, and
  `config/lattice/output.json` must match the costmap resolution (0.05).
- **LiDAR-only (`depth:=none`):** `kratos.launch.py` passes `use_nvblox:=false` to Nav2,
  which removes the nvblox local costmap plugin from a temporary params copy. The source
  YAML stays intact for camera runs.
- **CUDA GLIM (`glim_gpu:=true`):** select `config_odometry_gpu.json` in a temporary config
  copy; requires a CUDA-enabled GLIM and a patched overlay built with CUDA.
- **rclpy scripts:** `SignalHandlerOptions.NO` plus explicit SIGINT/SIGTERM handlers, so the Nav2
  goal is cancelled on exit; after `spin_until_future_complete` times out, poll `future.done()`.

## Review hints

- **Placeholders, not bugs:** `lidar_z` 0.60, camera mount (`cam_*`), 50×50 m grid, 1.0 m lattice
  arc radius, MPPI speeds and critic weights.
- **Not tested on the moving rover.** Tested on the Orin: full bring-up with both sensors (bench),
  the hardware-free tests.
- **Never commit:** `build/`, `install/`, `log/`, `maps/`, `.home/`, `*.pcd`, `*.pgm`, GLIM dumps.
- **Docs:** short and concrete, commands that can be pasted, no filler. `README.md` is setup and
  running only; everything else is in `docs/` (`FIELD_TEST.md` is the test-day runbook).
