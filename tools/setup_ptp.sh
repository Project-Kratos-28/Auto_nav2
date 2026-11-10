#!/bin/bash
# Host (the Orin): serve PTP time on the Ethernet port the MID-360 is plugged into, so the
# LiDAR's timestamps are in the same clock as everything else.
#
# Optional. Without PTP, livox_ros_driver2 already stamps each packet with the Orin's clock on
# arrival (src/comm/pub_handler.cpp, GetEthPacketTimestamp), which is good enough for GLIM,
# nvblox and Nav2 (measured: IMU 0.9 ms median, 7.5 ms max). With PTP the MID-360 stamps points
# itself, in the Orin's clock, without network/arrival jitter: slightly cleaner deskewing in GLIM.
# The MID-360 follows a PTP (IEEE 1588) master on its network automatically; ptp4l makes the
# Orin that master.
#
#   sudo tools/setup_ptp.sh <ethernet interface>        # e.g. eno1; see `ip -brief address`
#   sudo tools/setup_ptp.sh <iface> --persist           # also start it at every boot (systemd)
# Then check: tools/check_time_sync.py (offset of each sensor's stamps vs the system clock).
set -euo pipefail
IFACE="${1:?usage: sudo $0 <ethernet interface> [--persist]}"
PERSIST="${2:-}"
command -v ptp4l >/dev/null || apt-get install -y linuxptp
# linuxptp >= 4.0 calls the option serverOnly (older: masterOnly).
if ptp4l -v 2>/dev/null | grep -qE '^[4-9]'; then ROLE=serverOnly; else ROLE=masterOnly; fi
CONF=/etc/linuxptp/kratos_ptp4l.conf
mkdir -p /etc/linuxptp
cat > "$CONF" <<CONF
# Kratos: the Orin is the PTP master for the MID-360 (see kratos_glim/tools/setup_ptp.sh).
[global]
$ROLE              1
time_stamping         software
network_transport     UDPv4
delay_mechanism       E2E
CONF
echo "wrote $CONF ($ROLE, software timestamps, UDPv4 E2E)"
if [ "$PERSIST" = "--persist" ]; then
    cat > /etc/systemd/system/kratos-ptp4l.service <<UNIT
[Unit]
Description=PTP master for the Livox MID-360 on $IFACE (Kratos)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -f $CONF -i $IFACE
Restart=always

[Install]
WantedBy=multi-user.target
UNIT
    systemctl daemon-reload
    systemctl enable --now kratos-ptp4l.service
    echo "kratos-ptp4l.service enabled; logs: journalctl -u kratos-ptp4l -f"
else
    echo "running ptp4l in the foreground (Ctrl+C stops it; --persist installs a service)"
    exec ptp4l -f "$CONF" -i "$IFACE" -m
fi
