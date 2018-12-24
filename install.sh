#!/bin/bash
if [[ "$USER" = "root" ]]; then
    echo "Do not run install as root"
    exit 1
fi

RED=`tput setaf 196`
GREEN=`tput setaf 70`
STEP=`tput setaf 51`
WARNING=`tput setaf 214`
RESET=`tput sgr0`

# Flag to exit if a command causes an error.
set -e

# Request user and password.
sudo echo ""

# Get the script current directory
BASE_DIR="`dirname \"$0\"`"

# Setup logfile.
LOG_FILE="logsetup.log"

echo "${RED}       *@@@@@@@@@@@@     @@@@@@@@@@@@    @@@@@@@@@@@@@@  @@@@@@@@@@@@@@@     &@@@@@@@@@@@*
     @@@@@@@@@@@@@@@@ #@@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@@
    @@@@@@@@@@@@@@@@@#@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@
   *@@@@@@@  @@@@@@@ @@@@@@@  @@@@@@@ @@@@@@@@  @@@@@@@*@@@@@@  @@@@@@@@ @@@@@@@@ @@@@@@@@
   @@@@@@@          @@@@@@@  #@@@@@@@ @@@@@@@  @@@@@@@ @@@@@@  @@@@@@@@ #@@@@@@@  @@@@@@@
  @@@@@@@          @@@@@@@@@@@@@@@@@ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@    @@@@@@@@@@@@@@@@
 &@@@@@@@         *@@@@@@@@@@@@@@@@ &@@@@@@@@@@@@@@  &@@@@@@@@@@@@@@@  @@@@@@@@@@@@@@@@@
@@@@@@@@  @@@@@@@ @@@@@@@  @@@@@@@ @@@@@@@@         @@@@@@@@ @@@@@@@@ @@@@@@@  @@@@@@@@
@@@@@@@@@@@@@@@@ @@@@@@@  %@@@@@@@ @@@@@@@          @@@@@@@ #@@@@@@@ @@@@@@@@  @@@@@@@
@@@@@@@@@@@@@@@@&@@@@@@  @@@@@@@@ @@@@@@@          @@@@@@@ @@@@@@@@@ @@@@@@@  @@@@@@@
@@@@@@@@@@@@@  @@@@@@@@  @@@@@@@ #@@@@@@          %@@@@@@  @@@@@@@@ @@@@@@@  &@@@@@@@${RESET}

===========================================================================================
Installing Takin...
The process may take a while. If you're worried something
went wrong, check the logs ($LOG_FILE)
===========================================================================================
"

echo "${STEP}Installing Tools...${RESET}"
{
	# Install installation tools
	sudo apt update
	sudo apt install -y python-wstool python-rosdep ninja-build
}

if [[ ! -f "src/.rosinstall" ]]
then
    echo "${STEP}Merge ROS dependancies ...${RESET}"
    {
        # Install installation tools
        wstool init src
        wstool merge -t src "$BASE_DIR/takin.rosinstall"

        wstool update -t src
    }
else
    echo "${WARNING}SKIPPED Merge ROS dependancies ...${RESET}"
fi

echo "${STEP}Installing ROS Dependancies...${RESET}"
{
	if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]
    then
        sudo rosdep init
    fi

    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
}

echo "${STEP}Adding rules...${RESET}"
{
    sudo cp "$BASE_DIR/takin_bringup/udev/49-imu.rules" /etc/udev/rules.d/
	sudo cp "$BASE_DIR/takin_bringup/udev/56-orbbec-usb.rules" /etc/udev/rules.d/

	sudo service udev reload
	sudo service udev restart

    sudo addgroup $USER dialout
}

echo "
${GREEN}===========================================================================================
Takin installation successful.
===========================================================================================${RESET}
"

read -p "${WARNING}{Warning}${RESET} To complete your installation, you need to reboot your computer. Do you want to reboot now ? [Y/n]" -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]
then
    sudo reboot now
fi

exit