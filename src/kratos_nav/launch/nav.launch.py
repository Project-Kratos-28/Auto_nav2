"""Nav2 on the live GLIM map (ROS 2 Jazzy).

Starts the things Nav2 needs that nobody else publishes:
  1. static TF  base_link -> livox_frame      (where the LiDAR sits on the rover)
  2. static TF  base_link -> zed_camera_link  (where the ZED 2i sits; for nvblox)
  3. Nav2: controller, smoother, planner, behaviors, bt_navigator, waypoint_follower,
     velocity_smoother, and their lifecycle manager.

Deliberately NOT started: map_server (pcd2pgm publishes /map live) and AMCL
(GLIM publishes map->odom). Driver, angle filter, GLIM, pcd2pgm and the camera
pipeline (kratos_perception) are started by kratos_bringup/launch/kratos.launch.py (README.md).

Nav2's nodes are started here directly instead of through nav2_bringup's
navigation_launch.py: in Jazzy that file also starts route_server, collision_monitor
and docking_server, which need their own configuration and abort the bringup without it.
The seven nodes below are what the Humble version of this stack ran.

Run:  ros2 launch kratos_nav nav.launch.py
      ros2 launch kratos_nav nav.launch.py lidar_z:=0.62 cam_x:=0.35 cam_z:=0.45 cam_pitch:=0.30
"""
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

NAV2_NODES = [
    # (package, executable, node name, extra remappings)
    # Every velocity command goes through the velocity smoother: controller and behaviors
    # publish cmd_vel_nav, the smoother publishes the final /cmd_vel.
    ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_smoother', 'smoother_server', 'smoother_server', []),
    ('nav2_planner', 'planner_server', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
     [('cmd_vel', 'cmd_vel_nav'),
      ('cmd_vel_smoothed', LaunchConfiguration('cmd_vel_out'))]),
]


def maybe_disable_nvblox(context):
    """Let LiDAR-only runs start Nav2 without the camera costmap plugin."""
    if context.launch_configurations['use_nvblox'].lower() != 'false':
        return []
    with open(context.launch_configurations['params_file']) as f:
        data = yaml.safe_load(f)
    local = data['local_costmap']['local_costmap']['ros__parameters']
    local['plugins'] = [name for name in local['plugins'] if name != 'nvblox_layer']
    local.pop('nvblox_layer', None)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='kratos_nav_lidar_',
                                     delete=False) as f:
        yaml.safe_dump(data, f)
        path = f.name
    return [SetLaunchConfiguration('params_file', path)]


def generate_launch_description():
    pkg = get_package_share_directory('kratos_nav')
    arg = LaunchConfiguration

    # BT xml and lattice paths have to be absolute, and they differ per machine, so fill them
    # in at launch time instead of hardcoding them in the yaml. BOTH must be set:
    # the stock through-poses tree calls Spin, which we don't load, and
    # bt_navigator refuses to activate if any tree references a missing server.
    bt_dir = os.path.join(pkg, 'behavior_trees')
    params = RewrittenYaml(
        source_file=arg('params_file'),
        param_rewrites={
            'use_sim_time': arg('use_sim_time'),
            'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_no_spin.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'navigate_through_poses_no_spin.xml'),
            # Smac Lattice motion primitives (planner_server GridBased).
            'lattice_filepath': os.path.join(pkg, 'config', 'lattice', 'output.json'),
        },
        convert_types=True,
    )

    # PLACEHOLDER MOUNT. base_link is defined ON THE GROUND directly under the
    # rover's turning centre. lidar_x = how far forward of that the MID-360 is,
    # lidar_z = MID-360 height above the ground (tape measure to the centre of
    # the sensor). lidar_z also fixes where "ground" is for every height filter
    # (nav2_params.yaml obstacle heights, pcd2pgm_live.yaml Z band, and the nvblox
    # slice band, which kratos_perception computes from its own lidar_z argument).
    lidar_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_livox',
        arguments=['--x', arg('lidar_x'), '--y', '0.0', '--z', arg('lidar_z'),
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'])

    # PLACEHOLDER MOUNT for the ZED 2i. zed_camera_link is the camera's centre (between
    # the lenses), x forward, z up. cam_pitch > 0 tilts the camera DOWN (rad), so it sees
    # the ground in front of the rover. The ZED wrapper's URDF hangs the lens and optical
    # frames below zed_camera_link. Measure these, or better, calibrate them against
    # the LiDAR (target-based LiDAR-camera calibration).
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_zed',
        arguments=['--x', arg('cam_x'), '--y', arg('cam_y'), '--z', arg('cam_z'),
                   '--yaw', arg('cam_yaw'), '--pitch', arg('cam_pitch'), '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'zed_camera_link'])

    nav2 = [
        Node(package=pkg_name, executable=exe, name=name, output='screen',
             parameters=[params], remappings=remaps)
        for pkg_name, exe, name, remaps in NAV2_NODES
    ]
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': arg('use_sim_time'), 'autostart': True,
                     'node_names': [name for _, _, name, _ in NAV2_NODES]}])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cmd_vel_out', default_value='cmd_vel'),
        DeclareLaunchArgument('use_nvblox', default_value='true', description='false: LiDAR-only costmap'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.60'),
        DeclareLaunchArgument('cam_x', default_value='0.30', description='ZED forward of base_link (m), PLACEHOLDER'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.45', description='ZED height above ground (m), PLACEHOLDER'),
        DeclareLaunchArgument('cam_pitch', default_value='0.30',
                              description='ZED downward tilt (rad, ~17 deg), PLACEHOLDER'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        OpaqueFunction(function=maybe_disable_nvblox),
        lidar_tf,
        camera_tf,
        *nav2,
        lifecycle,
    ])
