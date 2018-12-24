# takin_bringup

#### Camera 3D

**Failed to open USB device**

Based on this thread https://3dclub.orbbec3d.com/t/orbbec-openi-problem-with-usb/279/6

If you have issues with opening the camera:

 * Generate a file with the name orbbec-usb.rules
 * add to the file: SUBSYSTEM=="usb", ATTR{idProduct}=="0404", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
 * $ sudo cp orbbec-usb.rules /etc/udev/rules.d/
 * unplug the camera
 * $ udevadm control --reload-rules
 * plug the camera
 
#### Lidar

In our configuration, the Lidar is connected by a Ethernet Port. The Lidar needed to be on the same network as the listener.

The Lidar is configured on the 192.168.1.155. In case, you need to change the ipaddress, you need to change: 

- The lidar internal ipaddress with [Sick Software](https://www.sick.com/us/en/downloads/software?q=%3Atyp1%3AConfiguration%2520software%3Atyp2%3ASOPAS%2520ET%3ADef_Type%3ADownload) 
- The hostname inside the sick_tim launch file
 
