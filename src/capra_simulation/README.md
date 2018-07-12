# capra_simulation

Capra-Takin's **capra_simulation** package is the Gazebo to ROS interfacing
stack for autonomous robot simulation.

### Dependencies

See [capra_simulation dependencies](doc/dependencies.md)

### Usage

$ *roslaunch capra_simulation simulation.launch*

By default, the launch files will open the Gazebo GUI and rviz GUI.

For more information on how to use rviz, see http://wiki.ros.org/rviz/UserGuide

### Published Topics

- /capra/simulation/odom
- /clock
- /cmd_vel
- /joint_states
- /move_base/current_goal
- /move_base/goal
- /move_base_simple/goal
- /tf
- /tf_static
