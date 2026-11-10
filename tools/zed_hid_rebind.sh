#!/bin/bash
# Host: make sure the ZED 2i's HID interface (IMU/sensors, 2b03:f881) is bound to usbhid.
# The ZED SDK detaches usbhid to talk to the sensors over libusb and does not always reattach it
# when it exits. The next container started then has no /dev/hidraw node for the ZED (containers
# copy /dev when they start). start.sh and stop.sh run this; it is a no-op when already bound.
# Rebinding needs root: it uses `sudo -n` and prints the command if sudo asks for a password.
set -u
for dev in /sys/bus/usb/devices/*; do
    [ "$(cat "$dev/idVendor" 2>/dev/null)" = "2b03" ] && [ "$(cat "$dev/idProduct" 2>/dev/null)" = "f881" ] || continue
    for iface in "$dev"/"$(basename "$dev")":*; do
        [ -d "$iface" ] || continue
        name=$(basename "$iface")
        drv=$(basename "$(readlink "$iface/driver" 2>/dev/null)" 2>/dev/null)
        if [ "$drv" = "usbfs" ]; then
            echo "ZED HID $name: in use by a running ZED SDK (usbfs); left alone"
        elif [ -n "$drv" ]; then
            echo "ZED HID $name: bound to $drv"
        elif echo -n "$name" | sudo -n tee /sys/bus/usb/drivers/usbhid/bind >/dev/null 2>&1; then
            echo "ZED HID $name: was unbound, rebound to usbhid"
        else
            echo "ZED HID $name: NOT bound. Run: echo -n $name | sudo tee /sys/bus/usb/drivers/usbhid/bind"
        fi
    done
done
