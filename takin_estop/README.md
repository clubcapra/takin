# takin_estop
## Overview
takin_estop is a simple node that provide a service to toggle on and off
 a logical estop. The program listen for service call and then turn on or 
 off the logical pin298 or the pin21 of the J1 header on the Jetson TX2.
 <br/>
 <br/>
 The program use the library [https://github.com/MengHuanLee/jetsonTX2GPIO]
 to help the jetson talk to it's GPIO ports.
 <br/>
 <br/>
The program is listed as  `/takin_estop` in the service list.
## Requirements
You only need to have the takin package to be able to access this services.
You should have catkin_make and source the workspace for that to work.
## Execution
There exists a launch file for this service located inside the `takin/takin_estop/launch` 
folder.
<br/>
<br/>
To run the service simply launch the launch file. 
<br/>
<br/>
`roslaunch takin_estop estop.launch`
### Testing
To the if the service is started you can use the `rosservice` tool to call
the service and see if there's a response. The command is this :
<br/>
<br/>
`rosservice call /takin_estop`
## Troubleshooting
If you're having issue with this package there might be two cause.
1. The GPIO pins are having permissions issues. There's a script at this
link [https://jkjung-avt.github.io/gpio-non-root/] where it explain how 
to change the permission to fix this issue. Then try to restart the node
to see if it worked.

Also it is worth noting that there's a udev rules that configure the 
initial permission parameter on the GPIO pin 298. This file can be found
 in the gpio folder of this package. This file should be place inside 
 the /etc/udev/rules.d/ folder. 
 
2. The ROS node is not behaving correctly, so there's a number of thing
you should try to fix this
    1. Did you compile and source properly?
    2. Are you able to see the services with `rosservice list`
    3. Restart?