# Configuration

## Heights: `lidar_z`

GLIM's `map`/`odom` origin is the LiDAR's **start** pose, so the ground is at `z = −lidar_z`.
Every obstacle height band is 0.2 m to 1.8 m above ground, i.e. `0.2 − lidar_z` to `1.8 − lidar_z`
(−0.40 / 1.20 at `lidar_z` 0.60). The value appears in four places; change them together:

| Where | What |
|---|---|
| `start.sh lidar_z:=` (→ `nav.launch.py`, `perception.launch.py`) | static TF; nvblox slice band (computed) |
| `src/kratos_nav/config/nav2_params.yaml` | `min_obstacle_height` / `max_obstacle_height`, both costmaps, twice each: on `obstacle_layer` (its own minimum defaults to 0.0, which drops everything below the LiDAR) and on its `lidar` source; plus the voxel grid's `origin_z` (= the minimum; 16 × 0.1 m voxels reach the maximum) |
| `src/pcd2pgm/config/pcd2pgm_live.yaml` | `thre_z_min` / `thre_z_max` |

Symptom of a wrong value: the floor shows as obstacles, or low obstacles are missing.

## Camera mount

`cam_*` must be measured, better calibrated against the LiDAR. A 1° pitch error moves the ground
by 3.5 cm at 2 m; ground that rises into the band becomes an obstacle.

## Other settings

| File | Setting |
|---|---|
| `nav2_params.yaml` | footprint 1.3 × 1.2 m (measured, + 0.05 m padding), LiDAR `obstacle_min_range` 0.8 m, Smac State Lattice (diff-drive primitives, 1.0 m arcs, forward only, pivots penalised), RotationShim (one 0.3 rad/s pivot per new goal if > 45° off) + MPPI (≤ 0.5 m/s, ≤ 0.5 rad/s, may turn in place), no reversing, velocity smoother deadband 0.05 m/s, local costmap 6×6 m (LiDAR + nvblox), local inflation scaling 5.0 (MPPI), global 1.0 (Smac), global costmap `/map` + LiDAR |
| `behavior_trees/*_no_spin.xml` | Jazzy default trees without Spin or BackUp (never reverse; pivots are slow) |
| `config/lattice/output.json` | Smac Lattice motion primitives; regenerate if the costmap resolution or arc radius changes (`config/lattice/README.md`) |
| `pcd2pgm_live.yaml` | fixed grid origin (−25, −25), 50×50 m, 0.05 m: size it to the whole arena; points outside are dropped. Radius filter 0.75 m / 2 neighbours (GLIM's map is voxelized at 0.5 m; tighter values erase walls) |
| `lidar_angle_filter/config/angle_filter.yaml` | masked sectors (full widths), `front_center_deg` if LiDAR +X isn't rover-forward; scan gate (`min_usable_points` 120 beyond `min_usable_range` 0.7 m, else the scan is dropped: GLIM aborts on an empty scan); elevation window (−90..90 on the rover; 5..53 for a handheld test) |
| `glim/glim_config/config.json` | CPU modules (odometry CPU, sub-mapping passthrough, pose-graph global mapping). `config_global_mapping_pose_graph.json`: `min_travel_dist` 8 m (stock 50 m never closes loops in small areas) |
| `glim/glim_config/config_ros.json` | `base_frame_id: base_link`, topics, extension modules (`libwaypoint_manager.so`) |
| `kratos_perception/config/zed2i_glim.yaml` | ZED: HD720, 15 Hz published, **tracking and TF off** (GLIM owns the pose) |
| `kratos_perception/config/nvblox_glim.yaml` | nvblox: `odom`, 3 cm voxels, no color, 5 m integration, clears > 8 m from `base_link` |
| `kratos_perception/config/ess_zed2i.yaml` | ESS: contrast stretch, TTA (vertical-flip agreement), confidence 0.1. `tta: false` doubles the rate |

## Laptop / remote GUI

`~/kratos_glim/ros_network.env` sets the container's ROS networking (restart the stack after editing):

| Variable | Value | Why |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | must equal the laptop's |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` | the laptop can see the rover |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | same DDS on both sides |
| `ROS_STATIC_PEERS` | (commented) | laptop IP, if the radio drops the multicast DDS uses for discovery |

On the laptop: same domain ID and RMW; Humble: `ROS_LOCALHOST_ONLY` unset; Jazzy:
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`. It needs `waypoint_interfaces` (this repo) to call the
waypoint services. View the rover with `rviz2 -d src/kratos_bringup/rviz/laptop.rviz`: only `/map`,
costmaps, TF, `/plan` and odometry, which are light over the radio. Point clouds, images and the
nvblox mesh are not (1280×720 color is tens of MB/s). "2D Goal Pose" sends a Nav2 goal.

To see point clouds on the laptop, start the rover with `cloudini:=true`: it also publishes
Cloudini-compressed `/glim_ros/points/compressed` and `/glim_ros/map/compressed`, decoded by the
laptop GUI. Details, contract and limits: [`CLOUDINI.md`](CLOUDINI.md).
Humble ↔ Jazzy is not officially supported by ROS; standard messages and these services work over
Fast DDS, but test before relying on it.

Check from the laptop: `ros2 node list` (shows `/glim_ros`, `/bt_navigator`, `/nvblox_node`, ...),
`ros2 service list | grep waypoint`.

Clocks: without PTP, the Livox driver stamps each packet with the Orin's clock when it arrives
(`GetEthPacketTimestamp`, `src/comm/pub_handler.cpp`), so all sensors share one clock.
`python3 tools/check_time_sync.py` shows every sensor's offset (LiDAR ~100 ms: a scan is stamped
at its start). `tools/setup_ptp.sh <iface>` makes the Orin a PTP master for the LiDAR; optional,
it only removes arrival jitter.
