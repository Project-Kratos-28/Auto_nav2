#!/usr/bin/env python3
"""Test harness: stands in for GLIM + pcd2pgm + the rover, so the
waypoint_mission -> Nav2 chain can be tested end to end without hardware.

Matches the REAL interfaces (see kratos_nav/test/README.md for the audit that
produced these numbers):
  - TF: map -> odom published DYNAMICALLY every tick (like glim_ros'
    rviz_viewer.cpp, "World -> Odom"), NOT as a one-shot static transform.
    A static transform never goes stale, which would hide a real GLIM TF
    stall from Nav2's transform_tolerance checks.
  - odom -> base_link integrated from /cmd_vel, published every tick.
  - /glim_ros/odom (nav_msgs/Odometry): nav2_params.yaml points
    bt_navigator/controller_server/velocity_smoother's odom_topic at this;
    the earlier fake never published it at all.
  - /map: FIXED 1000x1000 grid @ 0.05 m, origin (-25,-25), frame "map",
    QoS reliable + transient_local, cells only 0/100 (no -1) - matches
    pcd2pgm_live.yaml (fixed_width_m/height_m 50, map_resolution 0.05,
    fixed_origin_x/y -25). Republished every ~10 s, like pcd2pgm re-publishing
    as GLIM's /glim_ros/map periodically grows.
  - A wall obstacle sits between the start pose and wp1's straight-line path,
    so SmacPlannerLattice has to route around it (see WALL_* below).
  - /get_waypoint, /list_waypoints: real field names (waypoint_interfaces),
    real global service names (no ~/ namespace - see
    waypoint_manager_module.hpp). GetWaypoint.Response.found is False both for
    an unknown name and for one still pending (not yet bound to a submap
    world pose) - matches the real waypoint_manager, which only iterates
    `waypoints`, never `pending_waypoints`, in on_get.
  - A fake loop closure moves wp1 +0.5 m in x on its 4th query (unchanged from
    the original fake) and wp2 is reported "not found" for its first 3
    queries, then appears - exercises waypoint_mission.py's
    resolve_with_wait() pending-retry path. While pending, /list_waypoints
    reports it as "wp2 (pending)", like the real module, so the e2e run
    (which uses the list, no `waypoints` param) checks the suffix handling.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from waypoint_interfaces.srv import GetWaypoint, ListWaypoints

# --- Real /map geometry (pcd2pgm_live.yaml: fixed_origin_x/y -25,
# fixed_width_m/height_m 50, map_resolution 0.05 -> 1000x1000 cells) ---
MAP_RES = 0.05
MAP_ORIGIN_X = -25.0
MAP_ORIGIN_Y = -25.0
MAP_WIDTH = 1000
MAP_HEIGHT = 1000
MAP_REPUBLISH_PERIOD = 10.0  # pcd2pgm republishes whenever /glim_ros/map grows

# --- Wall obstacle: a finite wall segment the planner must go around, not
# a slot the planner must go through. It fully blocks the straight line from
# the start (0,0) to wp1 (3,0); the map is 50x50 m so there is plenty of
# room to route around either end. ---
WALL_X = (1.4, 1.6)
WALL_Y = (-2.0, 2.0)
# Half-width used ONLY for this test's own "did the robot touch the wall"
# check (footprint is +-0.37 m + 0.03 m padding in nav2_params.yaml, so a
# circle of this radius approximates the robot's body around its center).
ROBOT_COLLISION_RADIUS = 0.65


def world_to_grid(x, y):
    col = int((x - MAP_ORIGIN_X) / MAP_RES)
    row = int((y - MAP_ORIGIN_Y) / MAP_RES)
    return row, col


def build_map_data():
    data = bytearray(b'\x00' * (MAP_WIDTH * MAP_HEIGHT))
    r0, c0 = world_to_grid(WALL_X[0], WALL_Y[0])
    r1, c1 = world_to_grid(WALL_X[1], WALL_Y[1])
    for row in range(max(r0, 0), min(r1 + 1, MAP_HEIGHT)):
        base = row * MAP_WIDTH
        for col in range(max(c0, 0), min(c1 + 1, MAP_WIDTH)):
            data[base + col] = 100
    return bytes(data)


def robot_touches_wall(x, y):
    cx = min(max(x, WALL_X[0]), WALL_X[1])
    cy = min(max(y, WALL_Y[0]), WALL_Y[1])
    return math.hypot(x - cx, y - cy) < ROBOT_COLLISION_RADIUS


class FakeGlim(Node):
    def __init__(self):
        super().__init__('fake_glim')
        self.x = self.y = self.yaw = 0.0
        self.cmd = Twist()
        self.wp = {'wp1': [3.0, 0.0, 0.0], 'wp2': [3.0, 2.5, math.pi / 2]}
        self.wp1_queries = 0
        self.wp2_queries = 0
        self.collided = False

        # map -> odom is DYNAMIC (identity here; a real loop closure would
        # move it), published every tick alongside odom -> base_link, so a
        # stalled fake would show up as stale TF just like a stalled GLIM.
        self.tf = TransformBroadcaster(self)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.map_data = build_map_data()
        self.publish_map()
        self.create_timer(MAP_REPUBLISH_PERIOD, self.publish_map)

        self.odom_pub = self.create_publisher(Odometry, '/glim_ros/odom', 10)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_service(GetWaypoint, '/get_waypoint', self.on_get)
        self.create_service(ListWaypoints, '/list_waypoints', self.on_list)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = MAP_RES
        grid.info.width = MAP_WIDTH
        grid.info.height = MAP_HEIGHT
        grid.info.origin.position.x = MAP_ORIGIN_X
        grid.info.origin.position.y = MAP_ORIGIN_Y
        grid.info.origin.orientation.w = 1.0
        grid.data = list(self.map_data)
        self.map_pub.publish(grid)
        self.get_logger().info(
            f'published /map ({MAP_WIDTH}x{MAP_HEIGHT} @ {MAP_RES} m, origin '
            f'({MAP_ORIGIN_X},{MAP_ORIGIN_Y})), wall x={WALL_X} y={WALL_Y}')

    def on_cmd(self, msg):
        self.cmd = msg

    def step(self):
        # rclpy can tear down mid-timer-callback during shutdown; a publish
        # after that raises RCLError, which is just teardown noise, not a
        # real failure - swallow it so the log stays readable.
        try:
            self._step()
        except Exception:
            pass

    def _step(self):
        v, w = self.cmd.linear.x, self.cmd.angular.z
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw += w * self.dt

        if not self.collided and robot_touches_wall(self.x, self.y):
            self.collided = True
            self.get_logger().error(
                f'COLLISION: robot entered the wall at x={self.x:.2f} y={self.y:.2f}')

        now = self.get_clock().now().to_msg()

        map_odom = TransformStamped()
        map_odom.header.stamp = now
        map_odom.header.frame_id = 'map'
        map_odom.child_frame_id = 'odom'
        map_odom.transform.rotation.w = 1.0
        self.tf.sendTransform(map_odom)

        odom_base = TransformStamped()
        odom_base.header.stamp = now
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_link'
        odom_base.transform.translation.x = self.x
        odom_base.transform.translation.y = self.y
        odom_base.transform.rotation.z = math.sin(self.yaw / 2)
        odom_base.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf.sendTransform(odom_base)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        # Real GLIM's /glim_ros/odom child_frame_id is the auto-detected IMU
        # frame, NOT base_link (see rviz_viewer.cpp) - kept here for realism
        # even though nav.launch.py's static TF is base_link -> livox_frame.
        odom.child_frame_id = 'livox_frame'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    def on_get(self, req, res):
        if req.name == 'wp2':
            self.wp2_queries += 1
            if self.wp2_queries <= 3:
                self.get_logger().info(
                    f"'wp2' still pending (query {self.wp2_queries}/3)")
                res.found = False
                return res
        if req.name not in self.wp:
            res.found = False
            return res
        if req.name == 'wp1':
            self.wp1_queries += 1
            if self.wp1_queries == 4:
                self.wp['wp1'][0] += 0.5
                self.get_logger().warn('FAKE LOOP CLOSURE: wp1 moved +0.5 m in x')
        x, y, yaw = self.wp[req.name]
        res.found = True
        res.pose = PoseStamped()
        res.pose.header.frame_id = 'map'
        res.pose.pose.position.x = x
        res.pose.pose.position.y = y
        res.pose.pose.position.z = 0.6
        res.pose.pose.orientation.z = math.sin(yaw / 2)
        res.pose.pose.orientation.w = math.cos(yaw / 2)
        return res

    def on_list(self, req, res):
        # Like the real waypoint_manager: pending tags are listed last, with a
        # " (pending)" suffix that /get_waypoint does not accept.
        pending = self.wp2_queries < 3
        res.names = [n for n in self.wp if not (pending and n == 'wp2')]
        if pending:
            res.names.append('wp2 (pending)')
        return res


def main():
    rclpy.init()
    node = FakeGlim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.get_logger().info(f'final robot pose x={node.x:.2f} y={node.y:.2f} yaw={math.degrees(node.yaw):.0f}deg')


if __name__ == '__main__':
    main()
