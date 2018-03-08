#!/bin/bash

# Verifies that the script is not ran as root.
if [ "$USER" = "root" ]; then
    echo "Do not run install.sh as root"
    exit 1
fi

allDone=0

abort()
{
	if [ $allDone -eq 0 ]
	then
	    echo "
=============================================
Error install Takin. Aborting.
Please check $logFile for details
=============================================" >&2
	fi
}

# Flag to exit if a command causes an error.
set -e

# Request user and password.
sudo echo ""

# Setup logfile.
logFile="logsetup.log"

echo "=============================================================
Installing Takin...
The process may take a while. If you're worried something
went wrong, juste check the logs ($logFile)
============================================================="

TAKIN_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

echo "Installing ROS..."
{
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

	sudo apt-get update -y
	sudo apt-get upgrade -y

	sudo apt-get install -y ros-kinetic-desktop-full -y

	if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]
    then
	echo "Rosdep already initialized, skipping... "
    else
	sudo rosdep init
	rosdep update
	rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
    fi


} >> $logFile

echo "Adding rules..."
{
	sudo cp $TAKIN_DIR/49-capra.rules /etc/udev/rules.d/
    sudo addgroup $USER dialout
}

echo "Installing Packages..."
{
	# Install installation tools
	sudo apt-get install git python-pip python-wstool

	# Install other ros-kinetic packages
	sudo apt-get install ros-kinetic-pointgrey-camera-driver ros-kinetic-scan-tools ros-kinetic-navigation ros-kinetic-joy ros-kinetic-serial ros-kinetic-lms1xx ros-kinetic-robot-localization ros-kinetic-hector-mapping ros-kinetic-hector-slam ros-kinetic-hector-mapping ros-kinetic-move-base-msgs ros-kinetic-move-base ros-kinetic-amcl ros-kinetic-astra-launch ros-kinetic-astra-camera 

	# ROS Vendor libs install
	rm -f $TAKIN_DIR/vendor/src/.rosinstall
	wstool init $TAKIN_DIR/vendor/src $TAKIN_DIR/rosinstall/capra.rosinstall
	wstool update -t $TAKIN_DIR/vendor/src
} >> $logFile


echo "Building workspace... This can take a while"

source /opt/ros/kinetic/setup.bash

cd $TAKIN_DIR/vendor/
catkin_make >> $logFile

cd $TAKIN_DIR
catkin_make >> $logFile

source $TAKIN_DIR/devel/setup.bash
cwd="$PWD"

echo "
=========================================================
Takin installation successful.
========================================================="

allDone=1

exit

