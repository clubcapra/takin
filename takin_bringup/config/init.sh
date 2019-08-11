#!/usr/bin/env bash
# Intialization script for the Takin projet. This script is call inside the /etc/rc.local script.

can_int="can 0"

# Module configuration for CANBus interfaces.
modprobe can
modprobe mttcan
modprobe can_raw
modprobe can_dev

# Setting up CANBus interfaces.
ip link set can0 type can bitrate 1000000
ip link set can0 up
ip link set can0 type can restart-ms 100
ip link set can0 type can restart

# Initializing the drives
cangen -n 10 -v -v ${can_int} > /dev/null

exit 0