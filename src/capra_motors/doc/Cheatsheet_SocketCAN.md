# SocketCAN documentation

## Settings in the .bashrc file
    $ sudo modprobe can
    $ sudo modprobe can_raw
    $ sudo modprobe mttcan

    $ sudo ip link set can0 type can bitrate 1000000
    $ sudo ip link set can0 up

## Debugging
### When a CAN error occurs
  ` $ sudo ip link set can0 type can restart-ms 100`
  
  ` $ sudo ip link set can0 type can restart`

### To start communication on the CAN bus
  ` $ cangen can0`
      * Drives LED will flash Orange
### To monitor what's happening on the CAN bus
  ` $ cansniffer can0`

### To stop all communiations from/to the Jetson from/to the BUS.
   `$ sudo ip link set can0 down`
