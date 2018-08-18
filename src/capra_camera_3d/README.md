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

To interface with 3D camera in ROS, use following command:

    roslaunch capra_camera_3d capra_camera_3d.launch

After roslaunch, to check which topics are published by capra_camera_3d executable :

    rostopic list

**Published topics**

Published topics will be namespaced under the /capra/camera_3d/ and
the tf tree

**rviz**

If rviz isn't installed, install rviz :

    sudo apt-get install ros-kinetic-rviz

Launch rviz with command :

    rviz
  
In rviz graphical interface :
  
  Under Global Options, changed the Fixed Frame to one of the following :
    
    - camera_depth_frame
    - camera_depth_optical_frame
    - camera_link
    - camera_rgb_frame
    - camera_rgb_optical_frame

  Global Status should be "Global Status: Ok" with "Fixed Frame OK"
  
  ![Alt text](doc/display_pane.png "Display Pane")
  
  In rviz graphical interface :
  
    - Click on "Add" button below in the "Displays" left pane
    - In popup window, choose tab "by topic"
    - Choose topic to subscribe to and visualize camera data
    
  ![Alt text](doc/rviz_topics.png "Popup Window")
  
**Failed to open USB device**

Based on this thread https://3dclub.orbbec3d.com/t/orbbec-openi-problem-with-usb/279/6

If you have issues with opening the camera:

 * Generate a file with the name orbbec-usb.rules
 * add to the file: SUBSYSTEM=="usb", ATTR{idProduct}=="0404", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
 * sudo cp orbbec-usb.rules /etc/udev/rules.d/
 * unplug the camera
 * udevadm control --reload-rules
 * plug the camera
 
