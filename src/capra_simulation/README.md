# capra_simulation

Capra-Takin's **capra_simulation** package is the Gazebo to ROS interfacing
stack for autonomous robot simulation.

### Dependencies

See [capra_simulation dependencies](doc/dependencies.md)

### Usage

$ *roslaunch capra_simulation simulation.launch*

By default, the launch files will open the Gazebo GUI and rviz GUI.

For more information on how to use rviz, see http://wiki.ros.org/rviz/UserGuide<br />
For an overview of how to extend this package, see [capra_simulation overview](doc/overview.md)

### Published Topics

- /capra/simulation/odom
- /clicked_point
- /clock
- /cmd_vel
- /gazebo/link_states
- /gazebo/model_states
- /gazebo/parameter_descriptions
- /gazebo/parameter_updates
- /gazebo/set_link_state
- /gazebo/set_model_state
- /gazebo_gui/parameter_descriptions
- /gazebo_gui/parameter_updates
- /initialpose
- /joint_states
- /move_base/NavfnROS/plan
- /move_base/TrajectoryPlannerROS/cost_cloud
- /move_base/TrajectoryPlannerROS/global_plan
- /move_base/TrajectoryPlannerROS/local_plan
- /move_base/TrajectoryPlannerROS/parameter_descriptions
- /move_base/TrajectoryPlannerROS/parameter_updates
- /move_base/cancel
- /move_base/current_goal
- /move_base/feedback
- /move_base/global_costmap/costmap
- /move_base/global_costmap/costmap_updates
- /move_base/global_costmap/footprint
- /move_base/global_costmap/inflation_layer/parameter_descriptions
- /move_base/global_costmap/inflation_layer/parameter_updates
- /move_base/global_costmap/obstacle_layer/parameter_descriptions
- /move_base/global_costmap/obstacle_layer/parameter_updates
- /move_base/global_costmap/parameter_descriptions
- /move_base/global_costmap/parameter_updates
- /move_base/goal
- /move_base/local_costmap/costmap
- /move_base/local_costmap/costmap_updates
- /move_base/local_costmap/footprint
- /move_base/local_costmap/inflation_layer/parameter_descriptions
- /move_base/local_costmap/inflation_layer/parameter_updates
- /move_base/local_costmap/obstacle_layer/parameter_descriptions
- /move_base/local_costmap/obstacle_layer/parameter_updates
- /move_base/local_costmap/parameter_descriptions
- /move_base/local_costmap/parameter_updates
- /move_base/parameter_descriptions
- /move_base/parameter_updates
- /move_base/result
- /move_base/status
- /move_base_simple/goal
- /odom
- /scan
- /tf
- /tf_static

