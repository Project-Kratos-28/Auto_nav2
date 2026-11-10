# glim_ros 1.2.2 `/glim_ros/map` fix (overlay)

## GLIM core guard for sensor gaps

On the Orin, a rare LiDAR/IMU gap left only one frame in GLIM's sliding window.
GLIM 1.2.2 then indexed the already marginalized previous frame and aborted with
`IndexedSlidingWindow: index out of range`. The patch
[`odometry_missing_imu_guard.patch`](odometry_missing_imu_guard.patch) keeps the
last pose when no previous frame or sufficient IMU data is available.

Build this separate overlay before the rover workspace (use `-DBUILD_WITH_CUDA=ON`
for a CUDA GLIM installation):

```bash
mkdir -p /opt/glim_core_fix_ws/src
git clone --branch v1.2.2 https://github.com/koide3/glim.git /opt/glim_core_fix_ws/src/glim
git -C /opt/glim_core_fix_ws/src/glim apply /path/to/auto_fastlio2/glim/glim_ros_fix/odometry_missing_imu_guard.patch
cd /opt/glim_core_fix_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select glim --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON
```

`build_bare.sh` and `bringup.sh` source this overlay before the patched
`glim_ros` overlay when it exists. Keep both overlays after the bare workspace.

**Why it's needed:** the apt package `ros-jazzy-glim-ros` 1.2.2 has a bug in
`rviz_viewer` that corrupts `/glim_ros/map` during a normal run.

1. The pose graph re-optimizes on an idle timer, and every time it fires
   `on_update_submaps`.
2. The viewer appends the latest submap again on each of those calls.
3. Its submap list then grows longer than the pose list, so poses are looked
   up out of range.
4. The published cloud fills with straight "sheets" of garbage points.
5. pcd2pgm turns them into **phantom walls** in `/map`, and Nav2 plans
   around walls that do not exist.

Upstream fixed this in koide3/glim_ros2#76 (after 1.2.2).
`rviz_viewer_submap_guard.patch` is that fix backported: one `if` that
appends only when a new submap actually exists. It is needed whenever
something consumes `/glim_ros/map` (pcd2pgm, and therefore Nav2). It does
not matter for mapping itself.

## Build (once per machine, about 5 min)

The overlay is a separate workspace, so the upstream clone never lands inside this
repo:

```bash
mkdir -p ~/glim_ros_fix_ws/src && cd ~/glim_ros_fix_ws/src
git clone --branch v1.2.2 https://github.com/koide3/glim_ros2.git
cd glim_ros2
git apply /path/to/auto_fastlio2/glim/glim_ros_fix/rviz_viewer_submap_guard.patch
cd ~/glim_ros_fix_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

For `glim_gpu:=true`, install the CUDA build of GLIM first and add
`-DBUILD_WITH_CUDA=ON` to that `colcon build` command. The GLIM GPU package and
the host CUDA runtime must be compatible.

It builds against the apt GLIM (`libglim` headers from the koide3 PPA, the same
ones `ros-jazzy-glim-ros` uses). Only `glim_ros` is rebuilt. For bare metal,
set `KRATOS_GLIM_OVERLAY=$HOME/glim_ros_fix_ws/install/local_setup.bash` before
running `build_bare.sh` and `start.sh --bare-metal`.

On the team VM this already exists at `~/Kratos/glim_ros_fix_ws`, as a git
worktree of `~/Kratos/glim_ros2` at `v1.2.2` (`fc72f46`) with the patch applied.

## Use

Source the overlay **last**, after ROS and this workspace, in the terminal that
starts GLIM:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/auto_fastlio2/install_bare/setup.bash  # waypoint_manager
source ~/glim_ros_fix_ws/install/local_setup.bash     # MUST be last
ros2 pkg prefix glim_ros    # must print .../glim_ros_fix_ws/install/glim_ros, NOT /opt/ros/jazzy
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath /path/to/auto_fastlio2/glim/glim_config)
```

To be sure the patched library is the one loaded, run this while GLIM is running:
`grep -E 'librviz_viewer|libglim_ros' /proc/$(pgrep -f glim_rosnode)/maps`
Both libraries must come from `glim_ros_fix_ws`.

## When to drop this

Once the apt GLIM is newer than 1.2.2 and contains #76, delete this folder and the
overlay `source` lines in `docs/`.
