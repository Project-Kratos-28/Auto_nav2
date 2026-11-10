#!/usr/bin/env python3
"""Shared helpers for the pcd2pgm live-mode test suite.

Not wired into CMake / colcon test - run these scripts directly with python3
against a pcd2pgm_node process that is already running with
config/pcd2pgm_live.yaml. See test/README.md.
"""
import math
import os
import time

os.environ.setdefault('ROS_DOMAIN_ID', '44')

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

# --- Fixed grid constants, must match config/pcd2pgm_live.yaml -------------
ORIGIN_X = -25.0
ORIGIN_Y = -25.0
RES = 0.05
WIDTH = 1000
HEIGHT = 1000
Z_MIN = -0.40
Z_MAX = 1.20

# All GLIM-voxel-aligned test coordinates (multiples of 0.5, which is itself a
# multiple of the 0.05 grid resolution) sit EXACTLY on a cell boundary. The C++
# side computes floor((point.x - fixed_origin_x_) / map_resolution_) with
# point.x/map_resolution_ as float and fixed_origin_x_ as double (implicit
# promotion), while this file uses Python double throughout - two different
# roundings of the same nominally-"exact" division, which can floor to
# different integers right at a boundary (e.g. 18.0/0.05 as float-promoted
# double is 359.999999..., not 360.0). Nudge every generated test point by a
# small, non-multiple-of-0.05 epsilon so it sits unambiguously inside a cell
# instead of on the seam. 0.013 is << the 0.5m voxel spacing (so it doesn't
# change which points count as radius-filter neighbors) and not a multiple of
# 0.05 (so it can't recreate the same boundary problem).
EPS = 0.013

GLIM_TOPIC = '/glim_ros/map'
MAP_TOPIC = '/map'


def glim_publisher_qos(depth=1):
    """Matches the REAL GLIM rviz_viewer publisher: reliable + transient_local."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def map_subscriber_qos(depth=1):
    """Matches pcd2pgm's own /map publisher QoS (reliable + transient_local, keep_last(1))."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def cell_index(x, y, res=RES, ox=ORIGIN_X, oy=ORIGIN_Y, w=WIDTH, h=HEIGHT):
    """Returns the flat data[] index for world (x,y), or None if outside the grid."""
    i = math.floor((x - ox) / res)
    j = math.floor((y - oy) / res)
    if 0 <= i < w and 0 <= j < h:
        return i + j * w
    return None


def make_cloud(points, frame_id='map', stamp=None):
    """points: (N,3) float array-like. Builds a dense PointCloud2 (x,y,z float32)."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.is_bigendian = False
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def room_wall_points(x0, y0, size=4.0, spacing=0.5, zs=(0.0,), jitter=0.0, seed=0):
    """Same geometry family as /tmp/fake_wall_publisher.py (verified to fully survive
    thre_radius=0.75/thres_point_count=2): a square room's 4 walls, points spaced
    `spacing` apart (GLIM voxel resolution), at world origin (x0,y0)."""
    rng = np.random.default_rng(seed)
    x0 += EPS
    y0 += EPS
    xs = np.arange(0, size + 1e-6, spacing)
    ys = np.arange(0, size + 1e-6, spacing)
    pts = []
    for x in xs:
        for z in zs:
            pts.append((x0 + x, y0 + 0.0, z))
            pts.append((x0 + x, y0 + size, z))
    for y in ys:
        for z in zs:
            pts.append((x0 + 0.0, y0 + y, z))
            pts.append((x0 + size, y0 + y, z))
    pts = np.array(pts, dtype=np.float64)
    if jitter:
        pts += rng.normal(0, jitter, pts.shape)
    return pts


def voxel_cluster(x0, y0, z, n=3, spacing=0.5):
    """An n x n voxel-spaced cluster centered at (x0,y0,z) - each interior/edge point
    has >=1 neighbor within 0.5m, enough to survive thre_radius=0.75/thres=2 (same
    logic as room_wall_points, verified via /tmp run). Used as a z-filter or
    isolated-obstacle control cluster."""
    pts = []
    half = (n - 1) / 2.0
    x0 += EPS
    y0 += EPS
    for i in range(n):
        for j in range(n):
            pts.append((x0 + (i - half) * spacing, y0 + (j - half) * spacing, z))
    return np.array(pts, dtype=np.float64)


class TestNode(Node):
    """One rclpy node providing a GLIM-like publisher and a /map latching subscriber."""

    def __init__(self, name='pcd2pgm_test_driver'):
        super().__init__(name)
        self.glim_pub = self.create_publisher(PointCloud2, GLIM_TOPIC, glim_publisher_qos())
        self.map_msgs = []  # all received OccupancyGrid, in order
        self.map_sub = self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map, map_subscriber_qos())

    def _on_map(self, msg):
        self.map_msgs.append(msg)

    def publish_cloud(self, points, frame_id='map'):
        self.glim_pub.publish(make_cloud(points, frame_id=frame_id))

    def wait_for_discovery(self, timeout=15.0):
        """Blocks until pcd2pgm_node's subscription to /glim_ros/map is discovered
        (DDS discovery can take several seconds, especially the first match under
        load) - avoids the first publish being lost/delayed past a short timeout."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.glim_pub.get_subscription_count() >= 1:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_new_map(self, timeout=6.0):
        """Blocks until a NEW /map message arrives (beyond what's already recorded).
        Returns it, or None on timeout."""
        start_count = len(self.map_msgs)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.map_msgs) > start_count:
                return self.map_msgs[-1]
        return None

    @property
    def last_map(self):
        return self.map_msgs[-1] if self.map_msgs else None


def spin_for(node, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)


def tail_log(path, n_chars=20000):
    try:
        with open(path, 'r', errors='replace') as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - n_chars))
            return f.read()
    except FileNotFoundError:
        return ''


def log_size(path):
    """Current byte size of a log file - use as a marker before triggering an
    event, then pass to wait_for_log_match(..., since=...) so a match is found
    in the NEW content only, not an earlier line with the same substring
    (e.g. repeated "Live map update took" lines)."""
    try:
        return os.path.getsize(path)
    except FileNotFoundError:
        return 0


def wait_for_log_match(path, pattern, timeout=5.0, poll=0.2, since=0):
    """Polls a log file until a line containing `pattern` appears AFTER byte
    offset `since`. Returns the first such matching line (in file order), or
    None on timeout."""
    import re
    t0 = time.time()
    rx = re.compile(pattern)
    while time.time() - t0 < timeout:
        try:
            with open(path, 'r', errors='replace') as f:
                f.seek(since)
                new_text = f.read()
        except FileNotFoundError:
            new_text = ''
        for line in new_text.splitlines():
            if rx.search(line):
                return line
        time.sleep(poll)
    return None


class Failures:
    def __init__(self):
        self.items = []

    def check(self, name, cond, detail=''):
        if cond:
            print(f'[PASS] {name}')
        else:
            print(f'[FAIL] {name} :: {detail}')
            self.items.append((name, detail))

    def finish(self):
        print('-' * 60)
        if self.items:
            print(f'{len(self.items)} FAILURE(S):')
            for name, detail in self.items:
                print(f'  - {name}: {detail}')
            return 1
        print('ALL CHECKS PASSED')
        return 0
