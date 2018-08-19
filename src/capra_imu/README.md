# capra_imu

 Capra-Takin's **capra_imu** package contains the launch file for publishing many topics from the VN-300 driver (see **Published topics** section) of Club Capra's rescue robot.

### Dependencies

See [capra_imu dependencies](doc/dependencies.md)

### Overview

For an overview on how to extend this package, see [capra_imu overview.md](doc/overview.md)

### Usage

First get permission for /dev access, for example : 

  -  `$ sudo -i`

  -  `$ roslaunch capra_imu imu.launch`
  
  -  `$ roslaunch capra_imu imu_visualization.launch`

In others terminals, you can observe multiple data from the IMU:
  - $ rostopic echo /capra/imu
  - $ rostopic echo /capra/gps
  - $ rostopic echo /capra/mag
  - $ rostopic echo /capra/pres
  - $ rostopic echo /capra/temp

**Published topics**

  - /capra/imu
  - /capra/gps
  - /capra/mag
  - /capra/pres
  - /capra/temp

