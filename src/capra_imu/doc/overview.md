## Overview

To publish data onto your custom topics, simply change the remappings
in [imu.launch](../launch/imu.launch).

Example :

`<remap from="vectornav/IMU" to="<your_custom_topic>" />`

To avoid some problems, it could be a good idea to make a udev rule for the wire of the IMU. In this case, you need to change the serial_port parameter from the node in [imu.launch](../launch/imu.launch).

Example:

 `<param name="serial_port" type="string" value="/dev/new_usb_name" />`
