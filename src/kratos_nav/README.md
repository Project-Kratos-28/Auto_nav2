# kratos_nav

Nav2 for the Kratos rover. It uses GLIM for localization and pcd2pgm's live
`/map`, and runs the autonomous phase of the mission.

| File | What it does |
|---|---|
| `launch/nav.launch.py` | Nav2 `navigation_launch.py` (**no** map_server or AMCL) plus the static TF `base_link -> livox_frame` (args `lidar_x`, `lidar_z`, default 0.60) |
| `config/nav2_params.yaml` | Global costmap static layer on `/map` (from pcd2pgm), 6x6 m rolling local costmap, Smac State Lattice (diff-drive primitives in `config/lattice/`, forward only, pivots penalised), RotationShim (one slow pivot per new goal) + MPPI, no reversing, `odom_topic: /glim_ros/odom` |
| `behavior_trees/*_no_spin.xml` | Default BTs with the Spin and BackUp recoveries removed (never reverse; pivots are slow) |
| `config/lattice/` | Smac Lattice motion primitives (`output.json`) and the generator config that made them. See `config/lattice/README.md` |
| `scripts/waypoint_mission.py` | Drives to GLIM waypoints in order with `navigate_to_pose`, or as one route with `navigate_through_poses` (`mode:=through`). Re-queries `/get_waypoint` every 2 s and re-sends the goal if the waypoint moved more than 0.3 m (loop closure) |
| `test/` | Hardware-free tests. See `test/README.md` |

## Run

The full bring-up order is in `docs/ARCHITECTURE.md`; health checks are in `docs/FIELD_TEST.md`.
In short:

```bash
source /opt/ros/humble/setup.bash
source <repo>/install/setup.bash
ros2 launch kratos_nav nav.launch.py            # before GLIM (provides base_link -> livox_frame)
# ... GLIM + pcd2pgm running, waypoints tagged ...
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp2']"
ros2 run kratos_nav waypoint_mission.py --ros-args -p mode:=through   # one route through all
```

With no `waypoints` param, the mission uses `/list_waypoints`. That is every
bound waypoint in tag order, then any still-pending ones. The `" (pending)"`
suffix is stripped from the names.

### waypoint_mission.py parameters

| Parameter | Default | Meaning |
|---|---|---|
| `mode` | `pose` | `pose`: one `navigate_to_pose` goal per waypoint (stops at each). `through`: one `navigate_through_poses` goal with all of them (one route, no stop at each; a waypoint counts as passed within 0.7 m) |
| `waypoints` | `['']` (= use the list) | Names, in order |
| `refresh_period` | 2.0 s | How often to re-ask GLIM where the current waypoint is |
| `replan_threshold` | 0.3 m | Re-send the goal if the waypoint moved more than this |
| `wait_for_waypoint_sec` | 30.0 s | How long to wait for a pending (unbound) waypoint |
| `stop_on_failure` | true | Abort the mission when one waypoint fails |
| `base_frame`, `sensor_frame` | `base_link`, `livox_frame` | GLIM returns the LiDAR pose; it is converted to `base_link` with the static TF |

Ctrl+C cancels the active Nav2 goal. A goal Nav2 accepts after the 5 s send
timeout is cancelled too.

## Placeholders (measure before trusting)

| Value | Where | Placeholder |
|---|---|---|
| LiDAR height `lidar_z` | `nav.launch.py`, `src/pcd2pgm/config/pcd2pgm_live.yaml` (`thre_z_min/max`) | 0.60 m. **Change both together** |
| Terrain limits | `nav2_params.yaml` → `traversability_mapper` (`max_slope_deg`, `max_step`) | 20°, 0.15 m |
| Footprint | `nav2_params.yaml` (global and local costmap) | measured: 1.3 x 1.2 m (length x width) + 0.05 m padding |

## Tests

```bash
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh          # fake GLIM + real Nav2, ~2-4 min
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/mission_edge_test.sh # late-accept + Ctrl+C
```

Without `KRATOS_SETUP`, the shell tests source the VM workspaces
(`~/Kratos/glim_ext_ws`, `~/Kratos/kratos_nav_ws`). They use `ROS_DOMAIN_ID`
43 and 45, so run one at a time.
