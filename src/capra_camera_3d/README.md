# capra_camera_3d

 Capra-Takin's **capra_camera_3d** package is a wrapper for
 a FLIR lepton camera publishing point cloud data, infrared data,
 RGB data (see **Published topics** section) to Club Capra's rescue robot.

### Dependencies

See [capra_camera_3d dependencies](doc/dependencies.md)

### Overview

For an overview on how to extend/modify this package,
see [capra_camera_3d overview](doc/overview.md)

### Usage

First get permission for /dev access, for example : 

    $ sudo -i
    
    $ source <workspace>/devel/setup.bash

To interface with 3D camera in ROS, use following commands:

    $ roslaunch capra_camera_3d capra_camera_3d.launch

**rviz**

    $ roslaunch capra_camera_3d rviz.launch

**Published topics**

Published topics will be namespaced under the /capra/camera_3d/ and
the tf tree
  
**Failed to open USB device**

Based on this thread https://3dclub.orbbec3d.com/t/orbbec-openi-problem-with-usb/279/6

If you have issues with opening the camera:

 * Generate a file with the name orbbec-usb.rules
 * add to the file: SUBSYSTEM=="usb", ATTR{idProduct}=="0404", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
 * $ sudo cp orbbec-usb.rules /etc/udev/rules.d/
 * unplug the camera
 * $ udevadm control --reload-rules
 * plug the camera
 
