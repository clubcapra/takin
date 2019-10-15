# Takin 
<!-- [![Build Status](https://travis-ci.com/clubcapra/Takin.svg?branch=master)](https://travis-ci.com/clubcapra/Takin) -->

Capra-Takin is a ROS-based solution for managing and operating Club Capra's rescue robot.

## Technologies used
- Ubuntu 18.04LTS
- ROS melodic
- git + GitHub

## Getting Started

This section will help you get ready to work on the Takin project. It is supposed to give you the necessary information to understand ROS and how our projet is built. There's also documentation on how to launch and use our project.

### ROS Documentation \ Tutorial

The ROS documentation can be found at : [http://wiki.ros.org/](http://wiki.ros.org/)

It is a requirement to understand the basics of ROS and Linux to be proficient in the project. 

As an initiation task, we ask that you complete the series of tutorials offered by ROS : [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials). It is generally advised to stop at the roswtf tutorial.

<!-- No simulation for the moment therefore this section is useless -->
<!-- It is also recommanded to be familiar with Gazebo which is the simulation suite used in our project. The tutorial for Gazebo can be find at : [http://gazebosim.org/](http://gazebosim.org/). If you're assigned to a projet that contain simulation, it would be highly advise to at some Gazebo tutorial such as :
 
* Overview and Installation
* Understanding the GUI
* Model Editor
* Building Editor

Those can be find the beginner section of the guided tutorial.  -->

### Building our project

To build the projet the first time, you need to execute the `install.sh` which will configure, compile and regenerate the necessary bindings to use ROS with our nodes. After that you'll be able to use `catkin_make` and source your build manually.

Here are the steps to build our project : 

 ```sh
 mkdir -p catkin_ws/src
 cd catkin_ws/src
 git clone https://github.com/clubcapra/Takin takin
 cd ..
 ./src/takin/install.sh
```

You need to select the right setup file for your shell when sourcing.

```sh
source devel/setup.bash
```
### Installing dependencies
To install dependencies, go to the top directory of your catkin workspace where the source code of the ROS packages you'd like to use are. Then run:
```sh
rosdep install --from-paths src --ignore-src -r -y
```

### Executing a node
There are two option to launch a node.
- rosrun 
- roslaunch

If you just want to launch the robot for a demo, log into the robot using `ssh` then:
```sh
source ~/Code/takin_ws/devel/setup.bash
roslaunch takin_bringup takin_base.launch &
```

### Debugging
To debug a node the following options exits:
- Checking the basics

    | Issue | Command |
    |------|------|
    | Checking if the master is reacheable | `roswtf`
    | Checking if node exists | `rosnode list`|
    | Checking if topic exist | `rostopic list` |
    | Checking if message is published | `rostopic echo /your_topic_here`|
    | Checking the frequency of a message publication | `rostopic hz /your_topic_here` |

- If a permission error happens:

    | Issue | Command |
    |------|------|
    | Check if the device is available on your computer. (Ex: `takin_estop` won't run on anything else than a Jetson TX2) | `lsusb` or `ls /sys/class/gpio` or specific to your hw | 
    | Check if you have the proper udev copied in your folder.| `ls /etc/udev/rules.d/` |
    

## Contibution 

If you want to contribute to the projet, you can fork it and then PR to add or modify code. 

### Standard

At this moment, we don't have a coding standard, but it is something we want to implement in the future. For new code, we recommend using the [ROS CppStyleGuide](http://wiki.ros.org/CppStyleGuide#ROS_C.2B-.2B-_Style_Guide).
