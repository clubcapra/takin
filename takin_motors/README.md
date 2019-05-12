# Takin_motors
Takin_motors is a node that listen to a twist/cmd_vel message to control the drive which in then control the motors. 
It also publish informations about the motors. The motors use the CANBus interface on the development board of the 
Nvidia Jetson TX2.

## Setup
A typical setup to be up and ready to use needs 4 steps :

1. Setup on the Jetson : 
    1. Jetson must be powered on.
    2. The kernel modules must be enable.
    3. The interfaces must be configured. 
    4. The interfaces must be enable.
    
    This section is automated inside the takin_bringup/config/init.sh script.
2. Powering the boards
    1. The CANBus board need to be powered.
        1. The board have a ISO 1050 TX chip which is the transceiver for the CANBus interface. This chip need to be 
        powered on both side. See the [documentation](http://www.ti.com/lit/ds/symlink/iso1050.pdf) for more 
        information.
3. Powering the drives and motors
    1. Both the drives and motors need to be powered independently.
        1. Specially the drives need to be blinking and flashing red/off alternatively.
4. Detecting the CANBus
    1. Once everything is configure and powered. We can use the command  
    ```console
    cangen -v -v -n 10 can0 
    ```
    This command is use to generate random noise on the CANBus so that the drive will detect it.
    This operation will be automated in the future.
    
Now the setup should be done and you can start using takin_motor to control the robot.
    
## Dependencies
 We use the CTRE Phoenix API 
([ver 2018_v14](http://www.ctr-electronics.com/downloads/api/cpp/html/index.html)) to communicate to the drive and 
control the motors. 

takin_motors use the temp.msg message to publish the temperature of the drive. In the future it'll use the 
takin_remote_controller for manual navigation. 
##Troubleshooting

For troubleshooting, you can use the package [can-utils](https://github.com/linux-can/can-utils), which you can download
through apt. This package contain tools to interact with the CANBus interface.

