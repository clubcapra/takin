# Takin
 Capra-Takin is a ROS-based solution for managing and operating Club Capra's rescue robot.

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

To build our projet, you need to execute the `catkin_make` command which will compile and regenerate the necessary bindings to use ROS with our nodes.

Here are the steps to build our project : 

1. `git https://github.com/clubcapra/Takin.git Takin`
2. `cd Takin`
3. `./install.sh`
4. `catkin_make`
5. `source devel/setup.sh`

You need to select the right setup file for your shell when sourcing.

### Launch : Simulation

Before launching the projet you must have built it (this implies that you have also sourced your environment) once before. After this, you can use ROS to launch the simulation. 

`roslaunch capra_simulation simulation.launch`

### Launch : Real case

Right now the robot is not built, so there's no way to launch the robot in a real world scenario. 

## Contibution 

If you want to contribute to the projet, you can fork it and then PR to add or modify code. 

### Standard

Right now we don't really have any coding standard, but it is something we want to implement in the project. So the status of this part is TBD 

#### Code

#### Documentation
