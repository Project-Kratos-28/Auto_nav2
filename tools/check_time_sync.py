#!/usr/bin/env python3
"""Compare each sensor's message stamps with this machine's clock.

nvblox looks up GLIM's pose (stamped in LiDAR time) at each depth image's stamp (ZED: system
time), and Nav2 checks TF age against the system clock. All of them must share one clock.

  python3 tools/check_time_sync.py            # 10 s, default topics
Expected: every |offset| within a few tens of ms; the median is the sensor's latency
(LiDAR ~100 ms: a 10 Hz scan is stamped at its start). Measured on the Orin without PTP:
/livox/imu 0.9 ms, /livox/lidar 104 ms (the driver stamps packets with the Orin's clock on
arrival when the LiDAR is not PTP/GPS synced).
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

TOPICS = {
    '/livox/lidar': PointCloud2,
    '/livox/imu': Imu,
    '/glim_ros/odom': Odometry,
    '/tf': TFMessage,
    '/zed/zed_node/left/gray/rect/image': Image,
    '/ess/depth': Image,
}


def main(seconds=10.0):
    rclpy.init()
    node = Node('check_time_sync')
    offsets = {t: [] for t in TOPICS}

    def cb(topic):
        def f(msg):
            now = node.get_clock().now().nanoseconds
            stamps = [tr.header.stamp for tr in msg.transforms] if topic == '/tf' else [msg.header.stamp]
            for s in stamps:
                offsets[topic].append((now - (s.sec * 1_000_000_000 + s.nanosec)) / 1e6)
        return f
    for topic, typ in TOPICS.items():
        node.create_subscription(typ, topic, cb(topic), qos_profile_sensor_data)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    bad = False
    print(f'{"topic":<40}{"msgs":>6}{"median now-stamp":>20}{"max |offset|":>16}')
    for topic, v in offsets.items():
        if not v:
            print(f'{topic:<40}{0:>6}{"(no messages)":>20}')
            continue
        a = np.array(v)
        ok = np.abs(a).max() < 200.0
        bad |= not ok
        print(f'{topic:<40}{len(a):>6}{np.median(a):>17.1f} ms{np.abs(a).max():>13.1f} ms'
              + ('' if ok else '   <-- NOT on the system clock'))
    rclpy.shutdown()
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 10.0)
