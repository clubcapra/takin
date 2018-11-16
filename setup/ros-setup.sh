#!/bin/bash
echo "${step}Installing Tools...${reset}"
{
	# Install installation tools
	sudo apt-get install git python-wstool -y
}

echo "${step}Installing ROS...${reset}"
{
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

	sudo apt-get update -y

	if [[ "$fullVersion" = "true" ]]
	then
		sudo apt-get install ros-kinetic-desktop-full -y
	else
		sudo apt-get install ros-kinetic-ros-base -y
	fi
}

echo "${step}Installing ROS Dependancies...${reset}"
{
	source setup/ros-dependencies.sh
}

echo "${step}Adding rules...${reset}"
{
    sudo cp setup/49-capra.rules /etc/udev/rules.d/
	
	sudo cp setup/56-orbbec-usb.rules /etc/udev/rules.d/
	sudo service udev reload
	sudo service udev restart

    sudo addgroup $USER dialout
}

echo "${step}Adding ros environment to .bashrc...${reset}"
{
	if ! grep -q "source /opt/ros/kinetic/setup.bash" ~/.bashrc; then
		echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	fi

	if ! grep -q "source /opt/ros/kinetic/setup.zsh" ~/.zshrc; then
		echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
	fi
}

