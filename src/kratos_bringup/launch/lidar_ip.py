"""Find the MID-360's IP on the wired LiDAR subnet so the driver config never needs editing.

A MID-360 is 192.168.1.1xx, where xx comes from its serial, so the address differs
per unit (192.168.1.125, 192.168.1.162, ...). The host side stays 192.168.1.10.

How it works, with no root needed: sending a UDP datagram to an address makes the
kernel ARP for it. A device that answers becomes REACHABLE in the neighbour table.

Only REACHABLE counts. STALE/DELAY entries keep an old MAC for hours (the kernel does
not garbage-collect a table this small), so a unit that was unplugged or swapped would
still look present. A stale entry that is really alive turns REACHABLE after the
kernel's probe (~5-6 s), which is why the wait is 8 s.

Set LIVOX_LIDAR_IP to skip the search and force an address.
"""
import json
import os
import socket
import subprocess
import time

SUBNET = '192.168.1.'
HOST_IP = '192.168.1.10'
KNOWN_UNITS = ('192.168.1.162', '192.168.1.125')    # tried first, then the whole 1xx range
CANDIDATES = list(KNOWN_UNITS) + [
    f'{SUBNET}{i}' for i in range(100, 200) if f'{SUBNET}{i}' not in KNOWN_UNITS]
POKE_PORT = 9                                       # discard port: only the ARP matters


def reachable_neighbours(ip_neigh_output, candidates=CANDIDATES):
    """Candidate IPs that the kernel currently has as REACHABLE, from `ip neigh` text."""
    found = []
    for line in ip_neigh_output.splitlines():
        parts = line.split()
        if len(parts) < 2 or parts[0] not in candidates or parts[0] == HOST_IP:
            continue
        if 'lladdr' in parts and parts[-1] == 'REACHABLE':
            found.append(parts[0])
    return found


def _neigh():
    return subprocess.run(['ip', 'neigh'], capture_output=True, text=True).stdout


def pick(hits):
    """Prefer a known unit if several answered."""
    for ip in KNOWN_UNITS:
        if ip in hits:
            return ip
    return hits[0] if hits else None


def find_lidar_ip(wait_s=8.0, candidates=None):
    """Return the LiDAR's IP, or None if nothing answered on the subnet."""
    forced = os.environ.get('LIVOX_LIDAR_IP', '').strip()
    if forced:
        return forced
    candidates = candidates or CANDIDATES
    socks = []
    for ip in candidates:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setblocking(False)
        try:
            s.sendto(b'\x00', (ip, POKE_PORT))
        except OSError:
            pass
        socks.append(s)
    deadline = time.time() + wait_s
    hits = []
    try:
        while time.time() < deadline and not hits:
            time.sleep(0.25)
            hits = reachable_neighbours(_neigh(), candidates)
    finally:
        for s in socks:
            s.close()
    return pick(hits)


def write_config(template_path, out_path, lidar_ip):
    """Copy the driver's config with lidar_configs[*].ip replaced by lidar_ip."""
    with open(template_path) as f:
        cfg = json.load(f)
    for lidar in cfg['lidar_configs']:
        lidar['ip'] = lidar_ip
    with open(out_path, 'w') as f:
        json.dump(cfg, f, indent=2)
    return out_path


if __name__ == '__main__':
    # `python3 lidar_ip.py` prints the address (exit 1 if none): used by start.sh.
    ip = find_lidar_ip()
    print(ip or '')
    raise SystemExit(0 if ip else 1)
