# Configuration

## Heights: `lidar_z`

GLIM's `map`/`odom` origin is the LiDAR's **start** pose, so the ground is at `z = −lidar_z`.
The height bands (pcd2pgm's `/map`, nvblox's slice) are 0.2 m to 1.8 m above ground, i.e.
`0.2 − lidar_z` to `1.8 − lidar_z` (−0.40 / 1.20 at `lidar_z` 0.60). Change these together:

| Where | What |
|---|---|
| `start.sh lidar_z:=` (→ `nav.launch.py`, `perception.launch.py`) | static TF; nvblox slice band (computed) |
| `src/pcd2pgm/config/pcd2pgm_live.yaml` | `thre_z_min` / `thre_z_max` |

Symptom of a wrong value: the floor shows as obstacles in `/map`, or low obstacles are missing.
The costmaps' terrain layer needs no height: see below.

## Terrain: `traversability_mapper`

`nav2_params.yaml` → `traversability_mapper`. A 10 cm height map in `odom` from the LiDAR
(`/livox/lidar_filtered`, 1–10 m) and the ZED cloud, rated per cell and read by
`traversability_layer` in both costmaps:

| Rating | Rule | Grid value |
|---|---|---|
| lethal | drop > `max_step` (0.15 m) and steeper than `max_slope_deg` (20°) between two cells ≤ `short_radius` (0.5 m) apart, or up to `pair_radius` (2.5 m) across a gap the sensors can't see into (edge of a drop) | 100 |
| hidden | beams passed > `max_drop` (0.3 m) below the ground nearby, floor not seen yet | `hidden_cost` 60 |
| slope | 8° → 20° | 0 → 40 |
| unknown | never seen | −1 (costmap unchanged) |

`max_slope_deg` and `max_step` are placeholders: set them to what the rover can climb. The ZED
cloud comes at `point_cloud_freq` (1 Hz in `zed2i_glim.yaml`): the near field updates slowly.

## Camera mount

`cam_*` must be measured, better calibrated against the LiDAR. A 1° pitch error moves the ground
by 3.5 cm at 2 m; ground that rises into the band becomes an obstacle.

## Other settings

| File | Setting |
|---|---|
| `nav2_params.yaml` | footprint 1.3 × 1.2 m (measured, + 0.05 m padding), Smac State Lattice (diff-drive primitives, 1.0 m arcs, forward only, pivots penalised), RotationShim (one 0.3 rad/s pivot per new goal if > 45° off) + MPPI (≤ 0.5 m/s, ≤ 0.5 rad/s, may turn in place), no reversing, velocity smoother deadband 0.05 m/s, local costmap 6×6 m (terrain + nvblox), local inflation scaling 5.0 (MPPI), global 1.0 (Smac), global costmap `/map` + terrain |
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
