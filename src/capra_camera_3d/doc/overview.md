## Overview

The capra_camera_3d node is only a wrapper around
the astra_camera ROS package. Therefore, since the
astra_camera ROS package publishes many types of 
messages (See [capra_camera_3d readme](../README.md)),
we can choose to forward, through the subscribe_republish
method of NodeWrapper<TMessage>, any message of the topics
we choose to subscribe to. See [capra_camera_3d.cpp](../src/capra_camera_3d.cpp)'s comments for usage info.

Steps could be :

- `#include <ros_msg_type>`
- initialize `capra::NodeWrapper` object with `ros_msg_type` as template argument
- call subscribe_republish method with args (`ros::NodeHandle /*node*/`, `std::string /*topic_to_subscribe_to*/`, `std::string /*topic_to_republish_to*/`)

For complete information, see [NodeWrapper.h](../../../include/NodeWrapper.h)