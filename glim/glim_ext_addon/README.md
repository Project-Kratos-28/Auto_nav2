# Waypoint manager + headless dump exporter (GLIM extras)

Tags waypoints relative to the current submap's own origin instead of a frozen world
coordinate, so a tagged point automatically rides along with every pose-graph
loop-closure correction instead of going stale.

Depends only on `glim` and our own `waypoint_interfaces` package — no dependency on the
now-removed `FASTLIO2_ROS2/interface` package.

This folder also has `glim_dump_export`, a small standalone CLI that loads a saved
GLIM dump and writes straight to PCD — no GUI, no PLY intermediate. The installed
GLIM (1.2.2) `offline_viewer` doesn't have a command-line export flag (that was added
in a later GLIM version than the apt PPA ships), so this exists to make dump export
scriptable. Unlike `waypoint_manager`, it isn't a GLIM extension module and doesn't
need to go inside the `glim_ext` clone at all — it only needs `glim` itself.

## Build

`waypoint_interfaces`, `waypoint_manager`, and `glim_dump_export` build directly within this ROS 2 workspace:

```bash
cd /path/to/auto_fastlio2   # repo root; colcon finds these under glim/glim_ext_addon/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select waypoint_interfaces waypoint_manager glim_dump_export
```

To run unit tests on `waypoint_manager`:

```bash
colcon test --packages-select waypoint_manager --event-handlers console_direct+
colcon test-result --verbose
```

The tests cover tag -> resolve, loop-closure motion, and the pending-waypoint cases:
- a pending tag survives save and is listed as pending
- a pending name reports not-found
- a pending tag binds to the right submap

## Run

`libwaypoint_manager.so` is already configured in `extension_modules` in `glim/glim_config/config_ros.json`. Sourcing this workspace places `libwaypoint_manager.so` on `LD_LIBRARY_PATH`:

```bash
cd /path/to/auto_fastlio2   # repo root; colcon finds these under glim/glim_ext_addon/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath glim/glim_config)
```

## Services / topics

| Interface | Type | Purpose |
|---|---|---|
| `/add_waypoint` | `waypoint_interfaces/srv/AddWaypoint` | Tag the current pose as a named waypoint; it is queued until GLIM finalizes the submap containing that frame |
| `/get_waypoint` | `waypoint_interfaces/srv/GetWaypoint` | Resolve a named waypoint to its current `map`-frame pose, computed fresh from the submap's live-corrected position every call |
| `/list_waypoints` | `waypoint_interfaces/srv/ListWaypoints` | Names of everything tagged so far. Not-yet-bound tags are listed as `name (pending)` |
| `/save_waypoints` | `waypoint_interfaces/srv/SaveWaypoints` | Write all tagged waypoints to a YAML file. Pending tags go under `pending_waypoints` (odom pose + stamp) so a session that ends early does not lose them |
| `~/waypoints` | `visualization_msgs/MarkerArray` | Live-updated RViz markers (green sphere + text label), republished every 1s and whenever the pose graph re-optimizes |

Also autosaves to `/tmp/waypoints_autosave.yaml` every 10s while any waypoints
(bound or pending) exist, so a crash doesn't lose an entire session's tags.

`kratos_nav`'s `waypoint_mission.py` is the consumer. It calls `/get_waypoint`
every 2 s and retries while a name is still pending (see `src/kratos_nav`).

## Exporting a saved dump to PCD

```bash
source /opt/ros/humble/setup.bash
source /path/to/auto_fastlio2/install/setup.bash
ros2 run glim_dump_export glim_dump_export /path/to/saved/glim_dump /path/to/map.pcd "$(realpath /path/to/glim_config)"
```

Uses the dump's poses exactly as already optimized during the live run
(`enable_optimization=false`, same as GUI export) — does not add new loop closures.
Use `offline_viewer` instead if you need to manually close a missed loop or edit the
map before exporting.

## Known limitations

- **Single-session only.** Waypoints are stored `(name, submap_id, local_pose)` —
  submap-relative, not a resolved world coordinate — but there is no *load* path yet.
  Restarting GLIM starts a new map at a new origin; a saved YAML from a previous session
  cannot currently be reloaded into a new one. Loading requires solving submap
  correspondence between the old and new sessions (a relocalization problem), not yet
  implemented.
- `/add_waypoint` returns `success=false` until GLIM has processed an odometry frame.
- A tag made before its submap is finalized succeeds but stays **pending** until that
  submap closes. While pending:
  - `/get_waypoint` returns `found=false`.
  - `/list_waypoints` shows it as `name (pending)`.
  - save/autosave write it under `pending_waypoints` with its odometry-frame pose.
  - It is **not** loop-closure corrected yet.
- The pending entries in a saved YAML are for recovery by hand only. There is no load path
  (see above). Tag the last flag and keep driving a few metres so its submap finalizes.
