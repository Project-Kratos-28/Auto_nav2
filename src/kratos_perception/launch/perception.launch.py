"""ZED 2i depth -> nvblox (in GLIM's odom frame) -> /nvblox_node/static_map_slice for Nav2.

  ros2 launch kratos_perception perception.launch.py                  # ZED SDK NEURAL depth (default)
  ros2 launch kratos_perception perception.launch.py depth:=ess       # ESS depth
  ros2 launch kratos_perception perception.launch.py lidar_z:=0.62    # MUST match nav.launch.py

Needs, already running:
  GLIM                TF map -> odom -> base_link
  kratos_nav nav.launch.py  static TF base_link -> zed_camera_link (and -> livox_frame)
The ZED neither tracks nor publishes TF (config/zed2i_glim.yaml): GLIM owns the pose.

Obstacle band: nvblox marks surfaces between esdf_slice_min_height and esdf_slice_max_height
of its global frame (odom) as obstacles. GLIM's odom z=0 is the LiDAR's START height and the
ground is at -lidar_z, so the band "0.2 m to 1.8 m above ground" is computed here from lidar_z,
the same way nav2_params.yaml (min/max_obstacle_height) and pcd2pgm_live.yaml (thre_z_*) are.
Limits of a height band: a ramp steeper than ~10 deg shows as a wall about 1 m up it, and ditches
(negative obstacles) are not detected. See docs/ARCHITECTURE.md, Known limits.

Runs inside the kratos_glim container (docker/): nvblox, the ZED SDK and the ESS runtime
(~/kratos_nvblox/ess, mounted at /workspaces/kratos_nvblox) all live there.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ESS_ROOT = os.environ.get('KRATOS_ESS_ROOT', '/workspaces/kratos_nvblox/ess')

DEPTH = {  # depth:= option -> (ZED SDK depth mode, depth image, depth camera_info)
    'zed': ('NEURAL', '/zed/zed_node/depth/depth_registered', '/zed/zed_node/depth/camera_info'),
    'ess': ('NONE', '/ess/depth', '/ess/camera_info'),
}


def setup(context):
    arg = lambda name: context.launch_configurations[name]
    share = get_package_share_directory('kratos_perception')
    nvblox_share = get_package_share_directory('nvblox_examples_bringup')
    depth = arg('depth')
    if depth not in DEPTH:
        raise RuntimeError(f'depth:={depth} must be one of {list(DEPTH)}')
    zed_depth_mode, depth_image, depth_info = DEPTH[depth]

    lidar_z = float(arg('lidar_z'))
    band = {  # heights in odom: ground is at -lidar_z
        'esdf_slice_min_height': 0.2 - lidar_z,
        'esdf_slice_max_height': 1.8 - lidar_z,
        'esdf_slice_height': 0.2 - lidar_z,     # where the slice is drawn in RViz
    }

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'ros_params_override_path': os.path.join(share, 'config', 'zed2i_glim.yaml'),
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_urdf': 'true',      # zed_camera_link -> lens/optical frames (robot_state_publisher)
            'param_overrides': f'depth.depth_mode:={zed_depth_mode}',
        }.items())

    nvblox = ComposableNode(
        name='nvblox_node', package='nvblox_ros', plugin='nvblox::NvbloxNode',
        parameters=[
            os.path.join(nvblox_share, 'config/nvblox/nvblox_base.yaml'),
            os.path.join(nvblox_share, 'config/nvblox/specializations/nvblox_zed.yaml'),
            os.path.join(share, 'config', 'nvblox_glim.yaml'),
            {'num_cameras': 1, 'static_mapper': band},
        ],
        remappings=[
            ('camera_0/depth/image', depth_image),
            ('camera_0/depth/camera_info', depth_info),
            ('camera_0/color/image', '/zed/zed_node/rgb/color/rect/image'),
            ('camera_0/color/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
        ])
    actions = [
        zed,
        ComposableNodeContainer(
            name='nvblox_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', composable_node_descriptions=[nvblox], output='screen'),
    ]
    if depth == 'ess':
        actions.append(ExecuteProcess(
            cmd=[os.path.join(ESS_ROOT, '.venv/bin/python'), os.path.join(ESS_ROOT, 'ess_stereo_node.py'),
                 '--ros-args', '--params-file', os.path.join(share, 'config', 'ess_zed2i.yaml')],
            name='ess_stereo', output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='zed', description='zed (SDK NEURAL) | ess'),
        DeclareLaunchArgument('lidar_z', default_value='0.60',
                              description='MID-360 height above ground (m); MUST match nav.launch.py lidar_z'),
        OpaqueFunction(function=setup),
    ])
