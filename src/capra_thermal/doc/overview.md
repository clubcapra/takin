## Overview

The capra_thermal node is only a wrapper around
the FLIR Lepton's driver. Therefore, since the
driver's source code found publishes many types of 
messages (See [capra_thermal readme](../README.md)),
we can choose to forward, through the subscribe_republish
method of NodeWrapper<TMessage>, any message of the topics
we choose to subscribe to. See [capra_thermal.cpp](../src/capra_thermal.cpp)'s comments for usage info.

Steps could be :

- `#include <ros_msg_type>`
- initialize `capra::NodeWrapper` object with `ros_msg_type` as template argument
- call subscribe_republish method with args (`ros::NodeHandle /*node*/`, `std::string /*topic_to_subscribe_to*/`, `std::string /*topic_to_republish_to*/`)

For complete information, see [NodeWrapper.h](../../../include/NodeWrapper.h)
