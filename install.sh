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

echo "       *@@@@@@@@@@@@     @@@@@@@@@@@@    @@@@@@@@@@@@@@  @@@@@@@@@@@@@@@     &@@@@@@@@@@@* 
     @@@@@@@@@@@@@@@@ #@@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@@
    @@@@@@@@@@@@@@@@@#@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@ 
   *@@@@@@@  @@@@@@@ @@@@@@@  @@@@@@@ @@@@@@@@  @@@@@@@*@@@@@@  @@@@@@@@ @@@@@@@@ @@@@@@@@ 
   @@@@@@@          @@@@@@@  #@@@@@@@ @@@@@@@  @@@@@@@ @@@@@@  @@@@@@@@ #@@@@@@@  @@@@@@@  
  @@@@@@@          @@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@@@   
 &@@@@@@@         *@@@@@@@@@@@@@@@@ &@@@@@@@@@@@@@@  &@@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@@@   
@@@@@@@@  @@@@@@@ @@@@@@@  @@@@@@@ @@@@@@@@         @@@@@@@@ @@@@@@@@ @@@@@@@  @@@@@@@@    
@@@@@@@@@@@@@@@@ @@@@@@@  %@@@@@@@ @@@@@@@          @@@@@@@ #@@@@@@@ @@@@@@@@  @@@@@@@     
@@@@@@@@@@@@@@@@&@@@@@@  @@@@@@@@ @@@@@@@          @@@@@@@ @@@@@@@@@ @@@@@@@  @@@@@@@      
@@@@@@@@@@@@@  @@@@@@@@  @@@@@@@ #@@@@@@          %@@@@@@  @@@@@@@@ @@@@@@@  &@@@@@@@                      
===========================================================================================
Installing Takin...
The process may take a while. If you're worried something
went wrong, check the logs ($logFile)
===========================================================================================
"

TAKIN_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

read -p "Do you want to install the full-desktop version ? [Y/n]" -n 1 -r
echo

fullVersion=false
if [[ $REPLY =~ ^[Yy]$ ]]
then
    fullVersion=true
fi

echo "Installing Packages..."
{
	# Install installation tools
	sudo apt-get install git -y
}

echo "Installing ROS..."
{
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

	sudo apt-get update -y
	sudo apt-get upgrade -y

	if [ "$fullVersion" = "true" ]
	then
		sudo apt-get install ros-kinetic-desktop-full -y
	else
		sudo apt-get install ros-kinetic-ros-base -y
	fi
}

cd $TAKIN_DIR

echo "Installing ROS Dependancies..."
{
	if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]
    then
		sudo rosdep init
    fi

	rosdep update
	rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
}

echo "Adding rules..."
{
    sudo cp 49-capra.rules /etc/udev/rules.d/
    sudo addgroup $USER dialout
}

echo "Adding ros environment to .bashrc"
{
	if ! grep -q "source /opt/ros/kinetic/setup.bash" ~/.bashrc; then
		echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	fi

    source ~/.bashrc
}


echo "Building workspace... This can take a while"

source /opt/ros/kinetic/setup.bash

catkin_make >> $logFile

source devel/setup.bash

echo "
===========================================================================================
Takin installation successful.
==========================================================================================="

allDone=1

exit

