# Field test: map, tag waypoints, run the mission

Test-day procedure on the rover's Orin. Setup: [`README.md`](../README.md); settings: [`CONFIGURATION.md`](CONFIGURATION.md).

## 0. Before going out

- [ ] **Measure `lidar_z`**: MID-360 centre height above the ground. If it isn't 0.60, set
      min = 0.2 − lidar_z and max = 1.8 − lidar_z in
      `src/pcd2pgm/config/pcd2pgm_live.yaml` (`thre_z_min` / `thre_z_max`), and pass
      `lidar_z:=<value>` to `start.sh`. Example: 0.55 → −0.35 / 1.25.
- [ ] **Measure the ZED mount** (`cam_x`, `cam_y`, `cam_z` from `base_link` on the ground under the
      turning centre; `cam_pitch` down-tilt in rad) and pass them to `start.sh`.
- [ ] **Footprint** (measured 1.3 × 1.2 m, base_link at the centre): check nothing sticks out past it (arm, mast, antennas); `footprint` in `nav2_params.yaml`, both costmaps.
- [ ] **Map grid** covers the arena: `fixed_*` in `pcd2pgm_live.yaml` (default 50×50 m around the start).
- [ ] **Network**: `end0` is `192.168.1.10/24`, `ping 192.168.1.162` answers; laptop on the same
      `ROS_DOMAIN_ID` (`ros_network.env`).
- [ ] Workspace built after any change to a new file (`docker/run_container.sh docker/build_ws.sh`;
      edits to existing `.py`/`.yaml`/`.xml` need no rebuild, it is a symlink install).

## 1. Start

```bash
ssh <orin>
~/kratos_glim/start.sh lidar_z:=<m> cam_x:=<m> cam_z:=<m> cam_pitch:=<rad>
```

Keep the rover still until the log shows `initial IMU state estimation result`. Commands below run
in a container shell: `~/kratos_glim/docker/run_container.sh` (inside `tmux` over SSH).
On the laptop: `rviz2 -d src/kratos_bringup/rviz/laptop.rviz`.

## 2. Health checks (stop at the first failure)

| # | Check | Pass |
|---|---|---|
| 1 | `ros2 topic hz /livox/lidar_filtered` | ~10 Hz |
| 2 | `ros2 topic hz /glim_ros/odom` | ~10 Hz |
| 3 | `ros2 run tf2_ros tf2_echo map base_link` | z ≈ −lidar_z; changes when the rover moves |
| 4 | `ros2 run tf2_ros tf2_echo base_link livox_frame` | translation (lidar_x, 0, lidar_z) |
| 5 | `ros2 lifecycle get /bt_navigator` | `active [3]` |
| 6 | `ros2 topic hz /nvblox_node/static_map_slice` | ~10 Hz (not with `depth:=none`) |
| 7 | `python3 tools/check_time_sync.py` | every topic within a few tens of ms (LiDAR ~100 ms) |
| 8 | after driving a few metres: `ros2 topic echo /map --field info --once` | width 1000, height 1000, resolution 0.05, origin −25/−25; identical on every update |
| 9 | RViz local costmap | 6×6 m square around the rover; floor clear; obstacles near the camera appear |
| 10 | RViz `/map` / global costmap | walls where GLIM's map has walls; floor clear |

Floor full of obstacles (8–10): `lidar_z` or the camera mount is wrong.

## 3. Part A: map and tag

1. Drive slowly (< 0.5 m/s), smoothly, with textured surfaces in view.
2. At each waypoint, stop **facing the direction you want to arrive in**, then:
   ```bash
   ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
   ```
   Pass: `success=True` (`tagged in submap N`, or `queued until its submap is finalized`).
3. After the last tag, drive a few more metres so its submap finishes.
4. Return near the start and retrace your first few metres with the same heading: GLIM closes the
   loop (needs ≥ 8 m travelled). The map tightens and waypoint markers jump slightly: correct.
5. Check:
   ```bash
   ros2 service call /list_waypoints waypoint_interfaces/srv/ListWaypoints
   ros2 service call /get_waypoint waypoint_interfaces/srv/GetWaypoint "{name: wp1}"
   ```
   Pass: no `(pending)`, `found=True`, sensible x/y.
6. Optional backup: `ros2 service call /save_waypoints waypoint_interfaces/srv/SaveWaypoints "{path: /workspaces/kratos_glim/maps/waypoints.yaml}"`

**Don't restart the stack from here on**: waypoints only exist in the running GLIM session.

## 4. Part B: mission, rover stationary

Keep the wheels disconnected from `/cmd_vel`.

```bash
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
ros2 topic echo /cmd_vel        # second shell
```

Pass:
- `-> 'wp1' at (x, y)` with the x/y from `/get_waypoint`.
- A green path to wp1 in RViz that follows curves.
- `/cmd_vel` never has `linear.x < 0` (no reversing). `linear.x` ≤ 0.5 and |`angular.z`| ≤ 0.5. Pivots (`linear.x = 0`, `angular.z ≠ 0`) at the start of a goal (RotationShim, 0.3 rad/s) and occasionally from MPPI on tight turns.
- After ~15 s without progress Nav2 aborts: `failed 'wp1' (status 6)`. Expected, the rover didn't move.

Ctrl+C cancels the goal.

## 5. Part C: shadow mode (you are the motors)

Same command as Part B (or with `-p mode:=through`). Drive along the green path at walking pace.

Pass: `reached 'wp1'` within 0.5 m, then wp2; on a loop closure
`'wp1' moved X m (loop closure) - re-sending goal` and the path jumps; `mission complete`.
Slower than 0.5 m in 15 s aborts; rerun.

Real autonomy is the same mission with the wheels listening to `/cmd_vel`.

## 6. Write down

- measured `lidar_z` and camera mount; whether the floor stayed clear in both costmaps
- any waypoint that jumped, and by how much
- planner failures and where (tight spot, goal near a wall)
- whether pivots in place are OK on this ground (slip, stalls), and the tightest comfortable arc (Lattice primitives use 1.0 m: `config/lattice/README.md`)
- depth rate: `ros2 topic hz /zed/zed_node/depth/depth_registered` (ESS: `logs.sh ess_stereo`, `latency ms ... fill`)

## 7. Stop

`~/kratos_glim/stop.sh` (save waypoints first if wanted). The GLIM session goes to
`~/kratos_glim/maps/<date_time>`.

## Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `start.sh`: `ERROR: container ... is running the ZED` | Another stack holds the camera. Stop it, or `depth:=none` |
| `start.sh`: `no MID-360 answering` | `end0` address / cable. `ip -br addr show end0` must list `192.168.1.10/24` |
| No `/livox/lidar` | Wrong LiDAR picked (launch log: `MID-360 at ...`): force it with `LIVOX_LIDAR_IP=192.168.1.1XX ./start.sh`, XX = last two digits of its serial |
| GLIM log goes quiet, angle filter warns `Dropping scan` | LiDAR covered or blocked (< 120 points beyond 0.7 m): scans are held back so GLIM doesn't abort; uncover it |
| GLIM log: `Failed to lookup transform ... livox_frame ... base_link` | Static TF missing: `nav.launch.py` not up |
| GLIM crashes at start: `failed to initialize GLFW` | `gui:=true` without a display; use `gui:=auto`/`false` |
| `/map` never appears | GLIM publishes its map after its first submap: drive a few metres |
| `/map` size or origin changes between updates | Bug (the grid is fixed by design); note both values |
| pcd2pgm: `keeping the previous /map` every update | Nothing in the height band: `lidar_z` / `thre_z_*` wrong |
| Straight "sheets" or walls in `/map` where nothing exists | GLIM without the fix overlay: `ros2 pkg prefix glim_ros` must be `/opt/glim_ros_fix_ws/...` |
| Floor shows as obstacles | Height band wrong (`lidar_z`) or camera mount/pitch wrong |
| No nvblox obstacles | `ros2 topic hz /zed/zed_node/depth/depth_registered` (ESS: `/ess/depth`); `tf2_echo odom zed_left_camera_frame_optical` must resolve |
| ZED fails: `NOT VALID SERIAL NUMBER FOR SENSORS MODULE` | ZED HID interface unbound: `stop.sh`, `tools/zed_hid_rebind.sh`, `start.sh` |
| `add_waypoint` → service not available | GLIM not running or `waypoint_manager` not loaded (`load libwaypoint_manager.so` in the log) |
| `add_waypoint` → `no odometry yet` | GLIM gets no data: check 1–2 in section 2 |
| Mission: `'wp3' not resolvable yet`, then `not found` after 30 s | Its submap isn't finished: drive a few metres, rerun |
| Mission: `waypoint 'x' not found in GLIM` | Typo, or the stack was restarted (waypoints gone) |
| Mission: `Nav2 navigate_to_pose not available` | Nav2 not active: `ros2 lifecycle get /bt_navigator` |
| `failed to create plan` | Tagged heading impossible in a tight spot, or goal inside inflation: re-tag in the open |
| Global costmap empty but `/map` exists | QoS: `ros2 topic info /map -v`, both sides RELIABLE + TRANSIENT_LOCAL |
| Costmap warnings about TF extrapolation | GLIM lagging; raise `transform_tolerance` (0.3) in both costmaps |
| Laptop sees no rover nodes | Domain ID / discovery range (`ros_network.env`); try `ROS_STATIC_PEERS=<laptop IP>` |
