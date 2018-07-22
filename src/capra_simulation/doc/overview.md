## Overview

### Package file hierarchy

![Alt text](package_hierarchy.png "Package File Hierarchy")

### The launch folder

The launch folder contains mostly gazebo-ros interfacing nodes.

#### Gazebo.launch

Nested launches : 
- world.launch
- capra_odometry_simulation.launch

Spawned nodes :
- spawn_urdf (from external gazebo-ros package)

Overrideable args :
- robot_description : default is robotcapra.urdf of capra_simulation's models/urdf folder
- gui : default is true (opens gazebo visualization if set to true)
- initial_position : default is (0, 0, 0) in (x,y,z) and (0, 0, 0) for (roll, pitch, yaw)
- world_name : default is random.world in capra_simulation's worlds folder

#### world.launch



#### simulation.launch

