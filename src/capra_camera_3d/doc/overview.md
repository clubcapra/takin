## Overview

The capra_camera_3d node is only a wrapper around
the astra_camera ROS package. Therefore, since the
astra_camera ROS package publishes many types of 
messages under the /camera namespace, (See [capra_camera_3d readme](../README.md)),
we simply override the "camera" arg to "camera_3d"
under the "capra" namespace in [launch file](../launch/capra_camera_3d.launch).

See https://github.com/orbbec/ros_astra_launch/blob/master/launch/astra.launch for
complete overrideable args.
