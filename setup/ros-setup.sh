#!/bin/bash
echo "${step}Installing Tools...${reset}"
{
	# Install installation tools
	sudo apt-get install git -y
}

echo "${step}Installing ROS...${reset}"
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

#cd $TAKIN_DIR

echo "${step}Installing ROS Dependancies...${reset}"
{
	if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]
    then
		sudo rosdep init
    fi

	rosdep update
	rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
}

echo "${step}Adding rules...${reset}"
{
    sudo cp setup/49-capra.rules /etc/udev/rules.d/
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

