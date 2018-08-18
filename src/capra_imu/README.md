# capra_imu

 Capra-Takin's **capra_imu** package is a wrapper for publishing the topics linear_acceleration_covariance and linear_acceleration from the imu 
 (see **Published topics** section) of Club Capra's rescue robot.

### Dependencies

See [capra_imu dependencies](doc/dependencies.md)

### Usage

    For now, the imu is not focntional in Gazebo from Takin, but it is fonctional on the IBEX-1/2 
Capra computers on the branch Master. The command is :
	~/urial/src/capra_imu roslaunch IMU.launch
    It still possible to collect the data from the IMU in command line in every computers:

    First intall what is need

        1. sudo apt-get install ros-kinetic-octomap ros-kinetic-serial ros-kinetic-move-base-msgs ros-kinetic-joy

    Then

        1. cd ~/Urial
        2. git clone https://github.com/dawonn/vectornav.git
        3. catkin_make
        4. source devel/setup.sh
        5. roscore

    Another terminal
        1. roslaunch vectornav vectornav.launch

    Another terminal
        1. rostopic echo /vectornav/IMU

**Published topics**
    (These type of message are found in sensor_msgs/msg/Imu.msg)
    - /linear_acceleration_covariance
    - /linear_acceleration
