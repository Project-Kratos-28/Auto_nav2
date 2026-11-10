#!/usr/bin/env python3
"""pcd2pgm live-mode test suite, scenarios A-F and H.

Usage:
    python3 test_main.py <node_log_path>

Assumes a pcd2pgm_node is ALREADY RUNNING with config/pcd2pgm_live.yaml
(started by run_all.sh). Exits 0 if every check passes, 1 otherwise.
"""
import os
import re
import sys
import time

os.environ.setdefault('ROS_DOMAIN_ID', '44')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import rclpy

from common import (
    Failures, TestNode, ORIGIN_X, ORIGIN_Y, RES, WIDTH, HEIGHT, Z_MIN, Z_MAX, EPS,
    cell_index, room_wall_points, voxel_cluster, wait_for_log_match, tail_log, log_size,
)


def check_info(fail, label, msg, expect_frame='map'):
    info = msg.info
    fail.check(f'{label}: width==1000', info.width == WIDTH, f'got {info.width}')
    fail.check(f'{label}: height==1000', info.height == HEIGHT, f'got {info.height}')
    fail.check(f'{label}: resolution==0.05', abs(info.resolution - RES) < 1e-9, f'got {info.resolution}')
    fail.check(
        f'{label}: origin==(-25,-25)',
        abs(info.origin.position.x - ORIGIN_X) < 1e-9 and abs(info.origin.position.y - ORIGIN_Y) < 1e-9,
        f'got ({info.origin.position.x}, {info.origin.position.y})')
    fail.check(f'{label}: frame_id=="map"', msg.header.frame_id == expect_frame, f'got {msg.header.frame_id!r}')
    return info


def occupied_set(msg):
    data = msg.data
    return {i for i, v in enumerate(data) if v == 100}


def occ_count(msg):
    return sum(1 for v in msg.data if v == 100)


def main():
    node_log = sys.argv[1] if len(sys.argv) > 1 else '/tmp/pcd2pgm_node.log'
    rclpy.init()
    node = TestNode()
    fail = Failures()

    try:
        discovered = node.wait_for_discovery(timeout=15.0)
        fail.check('setup: pcd2pgm_node discovered our /glim_ros/map publisher', discovered, 'no subscriber matched within 15s')
        if not discovered:
            return fail.finish()

        # ------------------------------------------------------------------
        # Scenario A+B: grid info fixed & correct across a GROWING cloud;
        # wall points land in the correct cells.
        # ------------------------------------------------------------------
        print('=== Scenario A/B: fixed grid info across growth + cell correctness ===')
        room1 = room_wall_points(x0=5.0, y0=-10.0, size=3.0, spacing=0.5, zs=(0.0,))
        node.publish_cloud(room1)
        map1 = node.wait_for_new_map(timeout=6.0)
        fail.check('A: map1 received', map1 is not None, 'timed out waiting for /map')
        if map1 is None:
            return fail.finish()
        info1 = check_info(fail, 'A/map1', map1)

        room2_extra = room_wall_points(x0=-15.0, y0=10.0, size=3.0, spacing=0.5, zs=(0.0, 0.5))
        room2 = np.vstack([room1, room2_extra])  # cloud only GROWS, like real GLIM
        node.publish_cloud(room2)
        map2 = node.wait_for_new_map(timeout=6.0)
        fail.check('A: map2 received', map2 is not None, 'timed out waiting for /map')
        if map2 is None:
            return fail.finish()
        info2 = check_info(fail, 'A/map2', map2)

        fail.check(
            'A: grid identical across updates despite growth',
            (info1.width, info1.height, info1.resolution, info1.origin.position.x, info1.origin.position.y)
            == (info2.width, info2.height, info2.resolution, info2.origin.position.x, info2.origin.position.y),
            f'{info1} vs {info2}')

        # B: every room1 wall point's cell must be occupied in BOTH map1 and map2
        # (map2 proves old points are not lost when the cloud grows and is
        # re-rasterized from scratch).
        expected_idxs = set()
        oob = 0
        for x, y, _z in room1:
            idx = cell_index(x, y)
            if idx is None:
                oob += 1
                continue
            expected_idxs.add(idx)
        fail.check('B: room1 points all within fixed grid', oob == 0, f'{oob} unexpectedly out of bounds')

        occ1 = occupied_set(map1)
        occ2 = occupied_set(map2)
        missing1 = expected_idxs - occ1
        missing2 = expected_idxs - occ2
        fail.check('B: room1 wall cells occupied in map1', not missing1, f'{len(missing1)} missing, e.g. {list(missing1)[:5]}')
        fail.check('B: room1 wall cells STILL occupied in map2 (grown cloud)', not missing2, f'{len(missing2)} missing, e.g. {list(missing2)[:5]}')

        # interior of room1 (well clear of any wall) must be free
        interior_idx = cell_index(5.0 + EPS + 1.5, -10.0 + EPS + 1.5)
        fail.check('B: room1 interior cell is free', map1.data[interior_idx] == 0, f'got {map1.data[interior_idx]}')

        # ------------------------------------------------------------------
        # Scenario C: z passthrough - below-ground and above-ceiling clusters
        # must NOT become obstacles, even though they'd survive radius filter.
        # ------------------------------------------------------------------
        print('=== Scenario C: z passthrough filter (ground/ceiling rejection) ===')
        floor_cluster = voxel_cluster(15.0, 15.0, Z_MIN - 0.20, n=3, spacing=0.5)   # z=-0.60, below thre_z_min
        ceil_cluster = voxel_cluster(-15.0, 15.0, Z_MAX + 0.30, n=3, spacing=0.5)    # z=1.50, above thre_z_max
        control_cluster = voxel_cluster(15.0, -15.0, 0.0, n=3, spacing=0.5)          # z=0.0, inside range

        cloud_c = np.vstack([floor_cluster, ceil_cluster, control_cluster])
        node.publish_cloud(cloud_c)
        map_c = node.wait_for_new_map(timeout=6.0)
        fail.check('C: map received', map_c is not None, 'timed out')
        if map_c is not None:
            floor_idx = cell_index(15.0 + EPS, 15.0 + EPS)
            ceil_idx = cell_index(-15.0 + EPS, 15.0 + EPS)
            ctrl_idx = cell_index(15.0 + EPS, -15.0 + EPS)
            fail.check('C: below-ground cluster center is FREE (z-filtered)', map_c.data[floor_idx] == 0, f'got {map_c.data[floor_idx]}')
            fail.check('C: above-ceiling cluster center is FREE (z-filtered)', map_c.data[ceil_idx] == 0, f'got {map_c.data[ceil_idx]}')
            fail.check('C: control cluster (in-range z) center IS occupied', map_c.data[ctrl_idx] == 100, f'got {map_c.data[ctrl_idx]}')

        # ------------------------------------------------------------------
        # Scenario D: isolated single noise points removed, voxelized walls survive.
        # ------------------------------------------------------------------
        print('=== Scenario D: radius-outlier noise rejection ===')
        noise_pts = np.array([
            (-20.0 + EPS, -20.0 + EPS, 0.0),
            (20.0 + EPS, 5.0 + EPS, 0.0),
            (2.0 + EPS, -20.0 + EPS, 0.0),
        ])
        wall_ctrl = room_wall_points(x0=-5.0, y0=-5.0, size=3.0, spacing=0.5, zs=(0.0,))
        cloud_d = np.vstack([noise_pts, wall_ctrl])
        node.publish_cloud(cloud_d)
        map_d = node.wait_for_new_map(timeout=6.0)
        fail.check('D: map received', map_d is not None, 'timed out')
        if map_d is not None:
            for (x, y, _z) in noise_pts:
                idx = cell_index(x, y)
                fail.check(f'D: isolated noise point ({x},{y}) removed', map_d.data[idx] == 0, f'got {map_d.data[idx]}')
            wall_idx = cell_index(-5.0 + EPS, -5.0 + EPS)
            fail.check('D: voxelized wall control point survives', map_d.data[wall_idx] == 100, f'got {map_d.data[wall_idx]}')

        # ------------------------------------------------------------------
        # Scenario E: points outside +-25m are dropped with a WARN, no crash.
        # ------------------------------------------------------------------
        print('=== Scenario E: out-of-bounds points (WARN, no crash) ===')
        # NOTE: isolated single OOB points get removed by the radius-outlier filter
        # before they ever reach the bounds check (same mechanism as Scenario D),
        # so they'd never trigger the WARN. Use small voxel CLUSTERS instead, so
        # they survive radius filtering and actually reach setMapTopicMsgFixedBounds.
        oob_cluster_1 = voxel_cluster(30.0, 0.0, 0.0, n=3, spacing=0.5)   # x=30 > 25
        oob_cluster_2 = voxel_cluster(-40.0, 10.0, 0.0, n=3, spacing=0.5)  # x=-40 < -25
        ctrl = voxel_cluster(0.0, 0.0, 0.0, n=3, spacing=0.5)
        cloud_e = np.vstack([oob_cluster_1, oob_cluster_2, ctrl])
        since = log_size(node_log)
        node.publish_cloud(cloud_e)
        map_e = node.wait_for_new_map(timeout=6.0)
        fail.check('E: map received (node did not crash)', map_e is not None, 'timed out - node may have crashed')
        line = wait_for_log_match(node_log, r'points fell outside the fixed grid bounds', timeout=5.0, since=since)
        fail.check('E: WARN logged for out-of-bounds points', line is not None, 'no matching WARN line found')
        if line:
            m = re.search(r'(\d+) points fell outside', line)
            dropped = int(m.group(1)) if m else -1
            fail.check('E: dropped count >= 18 (both 9-point OOB clusters)', dropped >= 18, f'log said {dropped}')
        if map_e is not None:
            ctrl_idx = cell_index(0.0 + EPS, 0.0 + EPS)
            fail.check('E: control cluster still rasterized correctly', map_e.data[ctrl_idx] == 100, f'got {map_e.data[ctrl_idx]}')

        # ------------------------------------------------------------------
        # Scenario F: empty cloud / all-filtered cloud handling.
        # pcd2pgm must NOT publish an all-free grid for these (that would wipe every
        # obstacle from Nav2's static layer): no new /map, and the latched one stays.
        # ------------------------------------------------------------------
        print('=== Scenario F: empty / all-filtered cloud ===')
        # First, make sure there IS an occupied map to potentially wipe.
        node.publish_cloud(room_wall_points(x0=8.0, y0=8.0, size=2.0, spacing=0.5, zs=(0.0,)))
        map_pre = node.wait_for_new_map(timeout=6.0)
        fail.check('F: pre-condition map (occupied) received', map_pre is not None, 'timed out')
        pre_occ = occ_count(map_pre) if map_pre is not None else -1
        print(f'    pre-condition occ count = {pre_occ}')

        # Empty cloud (0 points).
        node.publish_cloud(np.zeros((0, 3)))
        map_empty = node.wait_for_new_map(timeout=3.0)
        fail.check('F: empty cloud publishes no new /map (previous map kept)', map_empty is None,
                   f'got a new map with occ={occ_count(map_empty) if map_empty is not None else None} (was {pre_occ})')
        fail.check('F: latched map still has the previous obstacles after empty cloud',
                   node.last_map is not None and occ_count(node.last_map) == pre_occ,
                   f'last occ={occ_count(node.last_map) if node.last_map is not None else None}, expected {pre_occ}')

        # Re-establish an occupied map, then send an all-filtered cloud (all points
        # above the ceiling threshold) - same question, different filter stage.
        node.publish_cloud(room_wall_points(x0=8.0, y0=8.0, size=2.0, spacing=0.5, zs=(0.0,)))
        map_pre2 = node.wait_for_new_map(timeout=6.0)
        pre_occ2 = occ_count(map_pre2) if map_pre2 is not None else -1

        all_filtered = voxel_cluster(3.0, 3.0, 5.0, n=3, spacing=0.5)  # z=5.0, way above thre_z_max
        node.publish_cloud(all_filtered)
        map_filtered = node.wait_for_new_map(timeout=3.0)
        fail.check('F: all-filtered cloud publishes no new /map (previous map kept)', map_filtered is None,
                   f'got a new map with occ={occ_count(map_filtered) if map_filtered is not None else None} (was {pre_occ2})')
        fail.check('F: latched map still has the previous obstacles after all-filtered cloud',
                   node.last_map is not None and occ_count(node.last_map) == pre_occ2,
                   f'last occ={occ_count(node.last_map) if node.last_map is not None else None}, expected {pre_occ2}')

        # Node must still be responsive afterwards.
        node.publish_cloud(room_wall_points(x0=8.0, y0=8.0, size=2.0, spacing=0.5, zs=(0.0,)))
        map_recover = node.wait_for_new_map(timeout=6.0)
        fail.check('F: node recovers and processes a normal cloud afterwards', map_recover is not None, 'timed out')

        # ------------------------------------------------------------------
        # Scenario H: timing for a large (~500k point) cloud, from the log line.
        # ------------------------------------------------------------------
        print('=== Scenario H: large-cloud timing (~500k points) ===')
        xs = np.arange(-24.5, 24.5, 0.5)
        ys = np.arange(-24.5, 24.5, 0.5)
        zs = np.array([-0.35, 0.10, 0.55, 1.00])  # 4 layers inside [-0.40, 1.20]
        gx, gy, gz = np.meshgrid(xs, ys, zs, indexing='ij')
        base = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        n_passes = 14  # simulate repeated/overlapping GLIM submaps at ~same voxels
        rng = np.random.default_rng(1)
        passes = [base + rng.normal(0, 0.03, base.shape) for _ in range(n_passes)]
        big_cloud = np.vstack(passes).astype(np.float64)
        print(f'    generated {len(big_cloud)} points')

        since = log_size(node_log)
        t_pub = time.time()
        node.publish_cloud(big_cloud)
        map_big = node.wait_for_new_map(timeout=30.0)
        wall_clock_s = time.time() - t_pub
        fail.check('H: map received for large cloud', map_big is not None, 'timed out after 30s')

        line = wait_for_log_match(node_log, r'Live map update took', timeout=10.0, since=since)
        ms = None
        if line:
            m = re.search(r'took ([\d.]+) ms', line)
            ms = float(m.group(1)) if m else None
        fail.check('H: timing line found in log', line is not None, 'no "Live map update took" line found')
        if ms is not None:
            print(f'    processing time = {ms:.0f} ms (wall clock incl. pub/sub round trip = {wall_clock_s*1000:.0f} ms)')
            fail.check('H: processing time < 10000 ms (GLIM period)', ms < 10000.0, f'{ms} ms')
            if ms >= 5000.0:
                print(f'    WARNING: {ms:.0f} ms is over half of GLIM\'s 10s publish period - little margin')
        if map_big is not None:
            check_info(fail, 'H/big', map_big)

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return fail.finish()


if __name__ == '__main__':
    sys.exit(main())
