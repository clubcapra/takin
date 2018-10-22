#!/bin/zsh
source setup/env.sh

read "REPLY?Do you want to install the full-desktop version ? [Y/n]"

fullVersion=false
if [[ $REPLY =~ ^[Yy]$ ]]
then
    fullVersion=true
fi

echo "${fullVersion}"

source setup/ros-setup.sh

echo "${step}Building workspace... This can take a while${reset}"
{
    source /opt/ros/kinetic/setup.zsh

    catkin_make >> $logFile

    source devel/setup.zsh
}

echo "
${green}===========================================================================================
Takin installation successful.
===========================================================================================${reset}"

allDone=1

exit