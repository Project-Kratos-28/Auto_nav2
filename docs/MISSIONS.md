# Waypoints, missions and maps

## Waypoints and missions

Waypoints are tagged at the rover's current pose while GLIM runs. Call the services from any
shell in the container (`~/kratos_glim/docker/run_container.sh`) or from the laptop:

```bash
ros2 service call /add_waypoint waypoint_interfaces/srv/AddWaypoint "{name: wp1}"
ros2 service call /list_waypoints waypoint_interfaces/srv/ListWaypoints
ros2 service call /get_waypoint waypoint_interfaces/srv/GetWaypoint "{name: wp1}"
ros2 service call /save_waypoints waypoint_interfaces/srv/SaveWaypoints "{path: /workspaces/kratos_glim/maps/waypoints.yaml}"
```

- Stop facing the direction you want to arrive in: the waypoint's heading is the goal heading.
- A new tag is **pending** until GLIM finishes its submap (~every 5 m of travel): `/get_waypoint`
  says `found=False`, `/list_waypoints` shows `wp1 (pending)`, loop closure doesn't move it yet.
  After the last tag, keep driving a few metres.
- `/add_waypoint` fails with `no odometry yet` until GLIM has processed a frame.
- GLIM autosaves waypoints to `/tmp/waypoints_autosave.yaml` every 10 s.
- **Waypoints exist only in the running GLIM session.** GLIM 1.2.2 cannot relocalize in a saved
  map, and saved waypoint files cannot be loaded back. Restarting the stack starts a new map at a
  new origin. Keep it running from mapping through the mission.
- Waypoint markers: `/glim_ros/waypoints` (MarkerArray).

Drive the waypoints:

```bash
ros2 run kratos_nav waypoint_mission.py                                  # all tagged, in tag order, one at a time
ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1','wp3']"
ros2 run kratos_nav waypoint_mission.py --ros-args -p mode:=through      # one route through all
```

- `mode:=pose` sends one `navigate_to_pose` goal per waypoint and stops at each.
  `mode:=through` sends one `navigate_through_poses` goal; a waypoint counts as passed within 0.7 m.
- Every 2 s it re-reads the waypoints from GLIM; if one still ahead moved more than 0.3 m (loop
  closure), it re-sends the goal (through: with only the waypoints not yet passed).
- It waits up to 30 s for a pending waypoint. Ctrl+C cancels the Nav2 goal.
- Over SSH, run it inside `tmux`: a plain SSH session's processes are killed (SIGHUP) when the
  link drops. Parameters: [`src/kratos_nav/README.md`](../src/kratos_nav/README.md).

Test-day procedure with checks and troubleshooting: [`FIELD_TEST.md`](FIELD_TEST.md).

## Maps (GLIM sessions)

`stop.sh` files each session under `~/kratos_glim/maps/<date_time>/` (ignored by git): numbered
submaps, `graph.bin`, `values.bin`, `odom_*.txt` (trajectory before loop closure), `traj_*.txt`
(after), and the config used. Tools, in the container:

```bash
ros2 run glim_ros offline_viewer --map_path /workspaces/kratos_glim/maps/<date_time>   # needs a display
ros2 run glim_ros map_editor                                                           # needs a display
ros2 run glim_dump_export glim_dump_export /workspaces/kratos_glim/maps/<date_time> map.pcd /workspaces/kratos_glim/glim/glim_config
```

- `offline_viewer`: add missed loop closures (right-click a submap sphere → `Loop begin`, another
  → `Loop end`), plane bundle adjustment, re-optimize, `File → Save → Export Points` (PLY).
- `glim_dump_export`: dump → PCD without a GUI, using the poses as optimized live.
- A Nav2 map file from a PCD: `pcd2pgm_node` in file mode (`src/pcd2pgm/config/pcd2pgm.yaml`,
  set `pcd_file`), then `ros2 run nav2_map_server map_saver_cli -f <name>`.
