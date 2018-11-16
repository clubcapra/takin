#!/bin/zsh
source setup/env.sh

read "REPLY?Do you want to install the full-desktop version ? [Y/n]"

fullVersion=false
if [[ $REPLY =~ ^[Yy]$ ]]
then
    fullVersion=true
fi

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
===========================================================================================${reset}
"

read "REPLY?${warning}{Warning}${reset} To complete your installation, you need to reboot your computer. Do you want to reboot now ? [Y/n]"
echo

if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo reboot now
fi

exit