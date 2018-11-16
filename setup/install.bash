#!/bin/bash
source setup/env.sh

read -p "Do you want to install the full-desktop version ? [Y/n]" -n 1 -r
echo

fullVersion=false
if [[ $REPLY =~ ^[Yy]$ ]]
then
    fullVersion=true
fi

source setup/ros-setup.sh

echo "${step}Building workspace... This can take a while${reset}"
{
    source /opt/ros/kinetic/setup.bash

    catkin_make >> $logFile

    source devel/setup.bash
}

echo "
${green}===========================================================================================
Takin installation successful.
===========================================================================================${reset}
"

read -p "${warning}{Warning}${reset} To complete your installation, you need to reboot your computer. Do you want to reboot now ? [Y/n]" -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo reboot now
fi

exit