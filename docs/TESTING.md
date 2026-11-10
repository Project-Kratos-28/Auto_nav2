# Tests

In the container (`~/kratos_glim/docker/run_container.sh`), from `/workspaces/kratos_glim`:

```bash
colcon test --packages-select waypoint_manager lidar_angle_filter && colcon test-result --all
python3 src/kratos_bringup/test/test_lidar_ip.py                                          # MID-360 IP detection
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh                  # fake GLIM + real Nav2 + mission
MISSION_ARGS="-p mode:=through" KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/e2e_test.sh
KRATOS_SETUP=$PWD/install/setup.bash bash src/kratos_nav/test/mission_edge_test.sh         # late goal accept, Ctrl+C
PCD2PGM_SETUP=$PWD/install/setup.bash bash src/pcd2pgm/test/run_all.sh                     # pcd2pgm live mode
```

The shell tests use their own `ROS_DOMAIN_ID` (43–45) and `pkill` their node names: run them one at
a time, not during a live run. Details: [`src/kratos_nav/test/README.md`](../src/kratos_nav/test/README.md),
[`src/pcd2pgm/test/README.md`](../src/pcd2pgm/test/README.md).
