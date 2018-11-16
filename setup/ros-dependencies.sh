#!/bin/bash
if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]
then
    sudo rosdep init
fi

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

wstool update -t src