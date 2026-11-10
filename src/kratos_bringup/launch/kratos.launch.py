"""Whole Kratos rover stack in one command (Docker or host ROS 2 Jazzy).

  ros2 launch kratos_bringup kratos.launch.py                      # everything, ZED NEURAL depth, RViz
  ros2 launch kratos_bringup kratos.launch.py depth:=ess           # ESS depth instead of ZED NEURAL
  ros2 launch kratos_bringup kratos.launch.py depth:=none          # LiDAR only (no camera, no nvblox)
  ros2 launch kratos_bringup kratos.launch.py gui:=false           # headless (default when no local display)
  ros2 launch kratos_bringup kratos.launch.py cloudini:=true       # + Cloudini-compressed clouds for the laptop
  ros2 launch kratos_bringup kratos.launch.py lidar_z:=0.62 cam_x:=0.35 cam_z:=0.45 cam_pitch:=0.30

From the host, ~/kratos_glim/start.sh does the container part and passes arguments through.

Starts, in this order:
  1. livox_ros_driver2          /livox/lidar (PointCloud2, 10 Hz), /livox/imu (200 Hz)
  2. lidar_angle_filter         /livox/lidar_filtered (antenna sectors masked) for GLIM
  3. kratos_nav nav.launch.py   static TF base_link -> livox_frame, base_link -> zed_camera_link; Nav2
  4. GLIM (after 3 s, once the static TF exists)   TF map -> odom -> base_link, /glim_ros/map
     KEEP THE ROVER STILL until GLIM logs "initial IMU state estimation result" (~5 s).
  5. pcd2pgm (live)             /glim_ros/map -> /map for the global costmap
  6. kratos_perception          ZED 2i -> NEURAL (or ESS) -> nvblox -> local costmap nvblox_layer
  7. RViz                       rviz/kratos.rviz (only with gui)
  8. Cloudini (only with cloudini:=true)   /glim_ros/points/compressed, /glim_ros/map/compressed
     (point_cloud_interfaces/CompressedPointCloud2, cloudini_resolution, default 1 cm) for the
     laptop GUI's decoder (Auto_Gui kratos_cloudini). cloudini_ros 1.1.0 (apt) takes its QoS from
     the publishers it sees at start; started before GLIM, that is reliable + volatile, so
     /glim_ros/map/compressed is not latched (a late GUI gets the next map update).

gui:=auto (default) turns the GUIs on only when DISPLAY is a local X display (":1"), i.e. not over
SSH (unset, or "localhost:10.0" with X forwarding, which is far too slow for OpenGL over a radio).
Without gui, RViz is not started and GLIM runs without its OpenGL viewer: GLIM crashes at start
when that viewer cannot open a display ("failed to initialize GLFW", SIGSEGV). Its ROS map
publisher (librviz_viewer.so, /glim_ros/map for pcd2pgm) is kept. From the laptop, view the rover
with rviz/laptop.rviz instead (light topics only).

Not started: the mission (waypoint_mission.py). The stack ends at Nav2's /cmd_vel.
lidar_z is passed to nav.launch.py AND kratos_perception, so this is the one place to set it;
nav2_params.yaml and pcd2pgm_live.yaml still carry the matching height numbers (AGENTS.md).
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lidar_ip  # noqa: E402  (sibling module)

WS = os.environ.get('KRATOS_WORKSPACE', '/workspaces/kratos_glim')


def share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def gui_enabled(context):
    gui = context.launch_configurations['gui'].lower()
    if gui == 'auto':
        return os.environ.get('DISPLAY', '').startswith(':')
    return gui == 'true'


def glim_and_rviz(context):
    """GLIM with or without its OpenGL viewer, and RViz, depending on gui."""
    gui = gui_enabled(context)
    config = os.path.join(WS, 'glim', 'glim_config')
    gpu = context.launch_configurations['glim_gpu'].lower() == 'true'
    if not gui or gpu:
        # Config files have comments; edit text in a private copy, keeping the repo untouched.
        import tempfile
        prepared = tempfile.mkdtemp(prefix='kratos_glim_config_')
        for name in os.listdir(config):
            with open(os.path.join(config, name)) as f:
                text = f.read()
            if name == 'config_ros.json' and not gui:
                text = '\n'.join(line for line in text.split('\n') if '"libstandard_viewer.so"' not in line)
            if name == 'config.json' and gpu:
                text = text.replace('config_odometry_cpu.json', 'config_odometry_gpu.json')
            with open(os.path.join(prepared, name), 'w') as f:
                f.write(text)
        config = prepared
    actions = [TimerAction(period=3.0, actions=[Node(
        package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
        parameters=[{'config_path': config}])])]
    if gui:
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', share('kratos_bringup', 'rviz', 'kratos.rviz')]))
    return actions


def driver_config():
    """MID360_config.json with the IP of the MID-360 that answers (it differs per unit).

    LIVOX_LIDAR_IP forces an address (start.sh passes the one it found). The repo file is never edited.
    """
    cfg = share('livox_ros_driver2', 'config', 'MID360_config.json')
    ip = lidar_ip.find_lidar_ip()
    if not ip:
        print('[kratos.launch] WARNING: no MID-360 answered on 192.168.1.1xx (power, cable, Orin IP = host_net_info?); '
              'using the IP in MID360_config.json', flush=True)
        return cfg
    print(f'[kratos.launch] MID-360 at {ip}', flush=True)
    return lidar_ip.write_config(cfg, f'/tmp/kratos_MID360_config_{os.getuid()}.json', ip)


def generate_launch_description():
    arg = LaunchConfiguration
    mount_args = ['lidar_x', 'lidar_z', 'cam_x', 'cam_y', 'cam_z', 'cam_pitch', 'cam_yaw']

    driver = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,          # PointCloud2 (GLIM needs it; 1 = Livox CustomMsg)
            'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0, 'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': driver_config(),
        }])
    angle_filter = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share('lidar_angle_filter', 'launch', 'angle_filter.launch.py')))
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('kratos_nav', 'launch', 'nav.launch.py')),
        launch_arguments={**{a: arg(a) for a in mount_args},
                          'use_nvblox': PythonExpression(["'", arg('depth'), "' != 'none'"])}.items())
    pcd2pgm = Node(
        package='pcd2pgm', executable='pcd2pgm_node', name='pcd2pgm', output='screen',
        parameters=[share('pcd2pgm', 'config', 'pcd2pgm_live.yaml')])
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share('kratos_perception', 'launch', 'perception.launch.py')),
        launch_arguments={'depth': arg('depth'), 'lidar_z': arg('lidar_z')}.items(),
        condition=UnlessCondition(PythonExpression(["'", arg('depth'), "' == 'none'"])))
    # Started with the launch, before GLIM (3 s timer): seeing no publisher, the converter picks
    # reliable + volatile, which the laptop decoder (Auto_Gui kratos_cloudini) expects.
    cloudini = [Node(
        package='cloudini_ros', executable='cloudini_topic_converter', name=f'cloudini_{name}', output='screen',
        parameters=[{'compressing': True, 'topic_input': f'/glim_ros/{name}',
                     'topic_output': f'/glim_ros/{name}/compressed',
                     'resolution': ParameterValue(arg('cloudini_resolution'), value_type=float)}],
        condition=IfCondition(arg('cloudini')))
        for name in ['points', 'map']]

    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='zed', description='zed (SDK NEURAL) | ess | none'),
        DeclareLaunchArgument('gui', default_value='auto',
                              description='auto (on only with a local X display) | true | false'),
        DeclareLaunchArgument('cloudini', default_value='false',
                              description='true: also publish /glim_ros/{points,map}/compressed for the laptop'),
        DeclareLaunchArgument('cloudini_resolution', default_value='0.01',
                              description='Cloudini position resolution (m)'),
        DeclareLaunchArgument('glim_gpu', default_value='false',
                              description='true: use CUDA odometry (requires GPU GLIM build)'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='MID-360 forward of base_link (m)'),
        DeclareLaunchArgument('lidar_z', default_value='0.60', description='MID-360 height above ground (m)'),
        DeclareLaunchArgument('cam_x', default_value='0.30', description='ZED forward of base_link (m), PLACEHOLDER'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.45', description='ZED height above ground (m), PLACEHOLDER'),
        DeclareLaunchArgument('cam_pitch', default_value='0.30', description='ZED down-tilt (rad), PLACEHOLDER'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        driver, angle_filter, nav, pcd2pgm, perception, OpaqueFunction(function=glim_and_rviz), *cloudini,
    ])
