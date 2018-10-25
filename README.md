# Takin [![Build Status](https://travis-ci.com/clubcapra/Takin.svg?branch=master)](https://travis-ci.com/clubcapra/Takin)

 Capra-Takin is a ROS-based solution for managing and operating Club Capra's rescue robot.

## Features

This section will help you find what you need in Takin project. 

| Package | Description |
| ------ | ------ |
| [Simulation](src/capra_simulation/) | This package is the Gazebo to ROS interfacing stack for autonomous robot simulation.  |
| [Navigation](src/capra_navigation/) | This package is the ROS navigation stack setup for capra's autonomous robot navigation. |
| [QR Code](src/capra_qrcode_detection/) | This package is a simple wrapper around zbar_ros's barcode_reader, used to detect QR code data in real-time simulation. |
| [Landolt Detection](src/capra_landolt_detection) | This package is a detector that find gap in Landolt C in a camera feed.  |
| [CO2 Detection](src/capra_co2_detector) | This package is a ROS node to communicate with the Telaire T6703 CO2 sensor |
| [Remote](src/capra_remote_controller/) | This package is a wrapper for a remote controller.  |
| [Thermal](src/capra_thermal/) | This package is a wrapper around the FLIR Lepton thermal camera's ROS driver. |
| [Motor](src/capra_motors/) | This package is a wrapper around the motors |
| [IMU](src/capra_imu/) | This package contains the launch file for publishing many topics from the VN-300 driver |
| [Camera](src/capra_camera/) | This package is a wrapper around the camera. |
| [Camera3D](src/capra_camera_3d/) | This package is a wrapper around the camera 3D. |

## Getting Started

This section will help you get ready to work on the Takin project. It is supposed to give you the necessary information to understand ROS and how our projet is built. There's also documentation on how to launch and use our project.

### ROS Documentation \ Tutorial

The ROS documentation can be found at : [http://wiki.ros.org/](http://wiki.ros.org/)

It is a requirement to understand the basics of ROS and Linux to be proficient in the project. 

As an initiation task, we ask that you complete the series of tutorials offered by ROS : [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials). It is generally advised to stop at the roswtf tutorial.

It is also recommanded to be familiar with Gazebo which is the simulation suite used in our project. The tutorial for Gazebo can be find at : [http://gazebosim.org/](http://gazebosim.org/). If you're assigned to a projet that contain simulation, it would be highly advise to at some Gazebo tutorial such as :
 
* Overview and Installation
* Understanding the GUI
* Model Editor
* Building Editor

Those can be find the beginner section of the guided tutorial. 

### Building our project

To build the projet the first time, you need to execute the `install.sh` which will configure, compile and regenerate the necessary bindings to use ROS with our nodes. After you can just use `catkin_make` and source your `source devel/setup.bash` manually.

Here are the steps to build our project : 

 ```sh
$ git clone https://github.com/clubcapra/Takin
$ cd Takin
$ ./setup/install.bash
```

You need to select the right setup file for your shell when sourcing.

### Launch : Simulation

Before launching the projet you must have built it (this implies that you have also sourced your environment) once before. After this, you can use ROS to launch the simulation. 

```sh
$ roslaunch capra_simulation simulation.launch
```

### Launch : Real case

Right now the robot is not built, so there's no way to launch the robot in a real world scenario. 

## Contibution 

If you want to contribute to the projet, you can fork it and then PR to add or modify code. 

### Standard

Right now we don't really have any coding standard, but it is something we want to implement in the project. So the status of this part is TBD.
