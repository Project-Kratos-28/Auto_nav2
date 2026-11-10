"""Tests for the pure parts of launch/lidar_ip.py (no network, no ROS).

Run: python3 src/kratos_bringup/test/test_lidar_ip.py   (or with pytest)
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch'))
import lidar_ip  # noqa: E402

NEIGH = """192.168.64.1 dev enp0s1 lladdr 5a:1b:cd:00:11:22 REACHABLE
192.168.1.1 dev eth0 lladdr 00:11:22:33:44:55 REACHABLE
192.168.1.125 dev enp0s1  FAILED
192.168.1.162 dev enp0s1 lladdr e4:7a:2c:6e:fb:56 REACHABLE
192.168.1.10 dev enp0s1 lladdr ea:6a:bd:cf:2c:22 PERMANENT
192.168.1.130 dev enp0s1  INCOMPLETE
"""


def test_only_the_reachable_lidar_is_found():
    # the router at .1 is outside the MID-360 range, the others never answered
    assert lidar_ip.reachable_neighbours(NEIGH) == ['192.168.1.162']


def test_either_known_unit_is_found():
    assert lidar_ip.reachable_neighbours(
        '192.168.1.125 dev enp0s1 lladdr aa:bb:cc:dd:ee:ff REACHABLE') == ['192.168.1.125']


def test_stale_entries_do_not_count():
    # an old unit's MAC stays in the table as STALE/DELAY long after it is unplugged
    for state in ('STALE', 'DELAY', 'PROBE'):
        assert lidar_ip.reachable_neighbours(
            f'192.168.1.125 dev enp0s1 lladdr aa:bb:cc:dd:ee:ff {state}') == []


def test_nothing_resolved_gives_empty():
    assert lidar_ip.reachable_neighbours('192.168.1.125 dev enp0s1  FAILED\n') == []


def test_pick_prefers_a_known_unit():
    assert lidar_ip.pick(['192.168.1.140', '192.168.1.125']) == '192.168.1.125'
    assert lidar_ip.pick(['192.168.1.140']) == '192.168.1.140'
    assert lidar_ip.pick([]) is None


def test_env_override_skips_the_search():
    os.environ['LIVOX_LIDAR_IP'] = '192.168.1.177'
    try:
        assert lidar_ip.find_lidar_ip(wait_s=0.0) == '192.168.1.177'
    finally:
        del os.environ['LIVOX_LIDAR_IP']


def test_write_config_only_changes_the_lidar_ip():
    with tempfile.TemporaryDirectory() as d:
        src = os.path.join(d, 'in.json')
        with open(src, 'w') as f:
            json.dump({'MID360': {'host_net_info': {'cmd_data_ip': '192.168.1.10'}},
                       'lidar_configs': [{'ip': '192.168.1.125', 'pcl_data_type': 1}]}, f)
        out = lidar_ip.write_config(src, os.path.join(d, 'out.json'), '192.168.1.162')
        with open(out) as f:
            cfg = json.load(f)
    assert cfg['lidar_configs'][0] == {'ip': '192.168.1.162', 'pcl_data_type': 1}
    assert cfg['MID360']['host_net_info']['cmd_data_ip'] == '192.168.1.10'


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print('ok  ', t.__name__)
    print(f'{len(tests)}/{len(tests)} passed')
