# kratos_nav end-to-end test

`e2e_test.sh` runs a hardware-free end-to-end check of the autonomous
mission: it starts `fake_glim.py` (a stand-in for GLIM + pcd2pgm + the rover)
alongside real Nav2 (`nav.launch.py`) and the real
`scripts/waypoint_mission.py`, then asserts the mission drives to `wp1` then
`wp2`, in order, exit code 0. The waypoint names come from `/list_waypoints`
(no `waypoints` param).

## Run it

```
bash src/kratos_nav/test/e2e_test.sh                       # from the repo root
bash ~/Kratos/kratos_nav_ws/src/kratos_nav/test/e2e_test.sh  # VM runtime copy
```

By default the script sources `~/Kratos/glim_ext_ws/install/setup.bash` and
`~/Kratos/kratos_nav_ws/install/setup.bash` (the VM layout). Elsewhere, point it
at the workspace(s) that contain `waypoint_interfaces` and `kratos_nav`:
`KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh`.
The same applies to `mission_edge_test.sh`.

Only one instance at a time - the VM is shared (4 cores / 7.7 GB, other
agents run here too). Logs go to `/tmp/kratos_nav_e2e/` (`fake.log`,
`nav.log`, `mission.log`); override with `E2E_LOG_DIR=...`. `/tmp` is wiped
on reboot, this test directory is not.

Always set `ROS_DOMAIN_ID=43` for anything touching this test manually
(`ros2 topic echo`, etc.) - the script exports it itself, but a second
terminal poking at the same run needs to match it or it won't see anything.

## What `fake_glim.py` stands in for, and why those numbers

Audited against the real interfaces (`waypoint_interfaces` srv package,
`waypoint_manager_module.hpp`, `pcd2pgm_live.yaml`, `pcd2pgm.cpp`,
`glim_ros2/src/glim_ros/rviz_viewer.cpp`, `config_ros.json`):

| Interface | Real | Fake |
|---|---|---|
| `/map` grid | 1000x1000 @ 0.05 m, origin (-25,-25), frame `map` | same |
| `/map` QoS | reliable + transient_local, `keep_last(1)` | same |
| `/map` cell values | only 0 or 100, never -1 | same |
| `/map` republish | whenever `/glim_ros/map` grows (no fixed heartbeat) | every 10 s (approximates pcd2pgm's live cadence) |
| `/get_waypoint`, `/list_waypoints` | global names, no `~/` prefix | same |
| `GetWaypoint.Response.found` | `False` for unknown AND for a still-pending (unbound) name | same - `wp2` is reported not-found for its first 3 queries to exercise `waypoint_mission.py`'s pending-retry path |
| `ListWaypoints.Response.names` | bound names in tag order, then pending ones as `"<name> (pending)"` | same - `wp2` is listed as `wp2 (pending)` until it resolves. The e2e run passes no `waypoints` param, so the mission takes its names from this list and must strip the suffix |
| waypoint pose | LiDAR (`livox_frame`) pose in `map`, z = sensor height | same |
| TF `map -> odom` | published every frame (dynamic), can move on loop closure | published every tick via a regular (non-static) broadcaster - a static transform never goes stale, which would hide a GLIM TF stall from Nav2's `transform_tolerance` |
| TF `odom -> base_link` | continuous | integrated from `/cmd_vel` |
| `/glim_ros/odom` | `nav_msgs/Odometry`, `child_frame_id` = auto-detected IMU frame (NOT `base_link`) | published (the original fake never published this topic at all, despite `nav2_params.yaml`'s `bt_navigator`/`controller_server`/`velocity_smoother` all setting `odom_topic: /glim_ros/odom`); `child_frame_id` kept as a non-`base_link` value to match the real oddity |
| Obstacles | whatever GLIM has mapped | one wall segment, `x in [1.4, 1.6]`, `y in [-2.0, 2.0]` in `/map`, directly blocking the straight line from the start pose to `wp1` - forces `SmacPlannerLattice` to actually plan a detour instead of driving a straight line on an empty grid |
| Loop closure | pose graph correction moves a submap | `wp1` moves +0.5 m in x on its 4th `/get_waypoint` query (unchanged from the original fake) |

`fake_glim.py` also self-checks that the simulated robot pose never comes
within `ROBOT_COLLISION_RADIUS` (0.65 m, the footprint
half-width 0.60 m + 0.05 m padding in `nav2_params.yaml`) of the wall's occupied cells; a
violation logs `COLLISION: ...` to `fake.log`, which `e2e_test.sh` greps for
and fails on. This is a circle-vs-rectangle approximation of the rectangular
footprint, not exact.

## Known simplifications (not fixed - out of scope for this test)

- `map -> odom` is always identity here; a real loop closure would also
  rotate/translate it, not just shift the reported waypoint pose.
- No `/livox/lidar` obstacle points, so the local costmap's `obstacle_layer`
  and the global costmap's live `obstacle_layer` are never exercised - only
  the static layer (the wall) is.
- `/add_waypoint` and `/save_waypoints` are not implemented; unused by
  `waypoint_mission.py`.

## Other tests in this directory (added 2026-09-16)

| Test | What it proves | Needs |
|---|---|---|
| `bash mission_edge_test.sh` | `waypoint_mission.py` never leaves the rover driving: (A) Nav2 accepts a goal after the 5 s timeout → it gets cancelled; (B) Ctrl+C mid-goal → goal cancelled. Uses `fake_glim.py` + `fake_nav_server.py` (no real Nav2), domain 45, logs in `/tmp/kratos_nav_edge/` | VM |
