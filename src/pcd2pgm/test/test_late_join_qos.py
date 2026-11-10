#!/usr/bin/env python3
"""pcd2pgm live-mode test suite, scenario G: QoS (reliable+transient_local depth1,
~10s period) and late-joining subscriber behaviour.

Manages its OWN pcd2pgm_node process lifecycle (separate from test_main.py) because
part 2 needs to control start order precisely.

Usage:
    python3 test_late_join_qos.py <config_yaml_path> <logdir>

Exits 0 if every check passes, 1 otherwise.
"""
import os
import subprocess
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '44')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node

from common import (
    Failures, TestNode, glim_publisher_qos, map_subscriber_qos,
    make_cloud, room_wall_points, GLIM_TOPIC, MAP_TOPIC,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


def start_node(config_path, log_path):
    log_f = open(log_path, 'w')
    proc = subprocess.Popen(
        ['ros2', 'run', 'pcd2pgm', 'pcd2pgm_node', '--ros-args', '--params-file', config_path],
        stdout=log_f, stderr=subprocess.STDOUT, env=os.environ.copy())
    return proc, log_f


def stop_node(proc, log_f):
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except Exception:
        proc.kill()
    log_f.close()


def main():
    if len(sys.argv) < 3:
        print('usage: test_late_join_qos.py <config_yaml_path> <logdir>')
        return 2
    config_path, logdir = sys.argv[1], sys.argv[2]
    fail = Failures()

    # ------------------------------------------------------------------
    # Part 1: pcd2pgm's OWN /map publisher is reliable+transient_local
    # depth1 - a subscriber that joins AFTER publication still gets the
    # last grid immediately, without needing a new /glim_ros/map input.
    # ------------------------------------------------------------------
    print('=== G1: /map publisher QoS - late-joining subscriber gets latched data ===')
    rclpy.init()
    proc1, log1 = start_node(config_path, os.path.join(logdir, 'node_g1.log'))
    time.sleep(2.0)
    try:
        driver = TestNode('pcd2pgm_test_driver_g1')
        discovered = driver.wait_for_discovery(timeout=15.0)
        fail.check('G1: pcd2pgm_node discovered our publisher', discovered, 'no subscriber matched within 15s')
        driver.publish_cloud(room_wall_points(0.0, 0.0, size=2.0, spacing=0.5, zs=(0.0,)))
        first = driver.wait_for_new_map(timeout=6.0)
        fail.check('G1: initial /map received', first is not None, 'timed out')

        # New, independent subscriber, created well after the publish above,
        # with NO further /glim_ros/map input in between.
        late_node = Node('late_joiner_g1')
        received = {}

        def cb(msg):
            received['msg'] = msg
            received['t'] = time.time()

        late_node.create_subscription(OccupancyGrid, MAP_TOPIC, cb, map_subscriber_qos())
        t_sub = time.time()
        t_end = t_sub + 3.0
        while time.time() < t_end and 'msg' not in received:
            rclpy.spin_once(late_node, timeout_sec=0.1)
        fail.check(
            'G1: late-joining subscriber receives latched /map within 3s (no new input)',
            'msg' in received, 'no message received - transient_local latching may be broken')
        if 'msg' in received:
            dt = received['t'] - t_sub
            print(f'    late-joining subscriber got /map after {dt*1000:.0f} ms (no new /glim_ros/map published)')
        late_node.destroy_node()
        driver.destroy_node()
    finally:
        stop_node(proc1, log1)
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Part 2: cold-start gap. pcd2pgm's live_cloud_sub_ QoS is
    # rclcpp::QoS(5).reliable() - NO .transient_local(). If GLIM already
    # has a retained (transient_local) map published BEFORE pcd2pgm
    # (re)starts, does pcd2pgm pick it up immediately, or does it have to
    # wait for GLIM's next periodic publish?
    # ------------------------------------------------------------------
    print('=== G2: cold-start - does pcd2pgm get a pre-existing retained GLIM map? ===')
    rclpy.init()
    fake_glim = Node('fake_glim_g2')
    pub = fake_glim.create_publisher(PointCloud2, GLIM_TOPIC, glim_publisher_qos())
    period = 6.0  # shorter than real 10s to keep the test fast, still long enough to be conclusive
    cloud = room_wall_points(0.0, 0.0, size=2.0, spacing=0.5, zs=(0.0,))

    def publish_once():
        pub.publish(make_cloud(cloud))

    publish_once()
    fake_timer = fake_glim.create_timer(period, publish_once)
    # Let the retained sample sit for a bit before pcd2pgm even starts, like a
    # pcd2pgm restart mid-run.
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(fake_glim, timeout_sec=0.1)

    proc2, log2 = start_node(config_path, os.path.join(logdir, 'node_g2.log'))
    t_start = time.time()

    map_driver = Node('map_watcher_g2')
    received2 = {}

    def cb2(msg):
        if 't' not in received2:
            received2['t'] = time.time()
            received2['msg'] = msg

    map_driver.create_subscription(OccupancyGrid, MAP_TOPIC, cb2, map_subscriber_qos())

    t_end = t_start + period + 4.0
    while time.time() < t_end and 't' not in received2:
        rclpy.spin_once(fake_glim, timeout_sec=0.05)
        rclpy.spin_once(map_driver, timeout_sec=0.05)

    # NOTE ON INTERPRETATION: pcd2pgm.cpp's live_cloud_sub_ QoS is
    # rclcpp::QoS(5).reliable() with NO .transient_local() (default VOLATILE
    # durability) - see pcd2pgm.cpp:53-56. By the plain DDS durability rule this
    # should mean a freshly-started pcd2pgm cannot receive a sample GLIM already
    # published before pcd2pgm existed, and must wait for GLIM's next periodic
    # publish. This assertion checks that empirically, rather than assuming it.
    fail.check('G2: pcd2pgm produced a /map within the window (node alive & functional)', 't' in received2,
               'timed out - node may have crashed or is unresponsive')
    if 't' in received2:
        delay = received2['t'] - t_start
        print(f'    pcd2pgm produced its first /map {delay:.1f}s after process start '
              f'(fake GLIM publish period = {period:.0f}s, retained sample was already {1.5:.1f}s old at start)')
        if delay < period * 0.5:
            print('    FINDING: delivery was near-immediate despite the sub QoS lacking transient_local. '
                  'Likely explanation: GLIM/fake publisher uses RELIABLE + KEEP_LAST(1), so the writer '
                  'still holds that one outstanding sample and reliable delivery pushes it to any newly '
                  'matched reader regardless of the reader\'s requested durability - so no operational '
                  'gap was observed here. See report for caveats (this is RMW/implementation-dependent, '
                  'not guaranteed by the DDS spec).')
        else:
            print('    FINDING: delivery took roughly a full GLIM period - consistent with the sub QoS '
                  'gap (missing .transient_local()) actually mattering: a restarted pcd2pgm may show no '
                  '/map for up to ~10s in the real system.')

    fake_glim.destroy_node()
    map_driver.destroy_node()
    stop_node(proc2, log2)
    rclpy.shutdown()

    return fail.finish()


if __name__ == '__main__':
    sys.exit(main())
