## Overview

### Package file hierarchy

![Alt text](package_hierarchy.png "Package File Hierarchy")

### The launch folder

The launch folder contains mostly gazebo-ros interfacing nodes. 
The important ones to consider are the Gazebo.launch and
simulation.launch launch files.

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

#### simulation.launch



#### world.launch

The world.launch launch file is an almost identical copy of the gazebo-ros
package's empty_world.launch launch file. See the gazebo-ros package for
more info.

Overrideable args (only the most used ones) : 

- paused : default is false, set to true to start simulation in paused mode
- gui : default is true, set to true to open gazebo visualization
- recording : default is false, set to true to record (TODO : complete doc)
- debug : default is false, set to true for debugging information
- physics : default is ODE, can set to any other gazebo supported physics engine
- verbose : default is false, set to true to see gazebo messages stream in the console
- world_name : default is man1.world, set value to any other valid .world file
