# capra_remote_controller

 Capra-Takin's **capra_remote_controller** package is a wrapper for
 a Logitech remote controller publishing the topics joy and capra_remote_controller 
 (see **Published topics** section) to Club Capra's rescue robot.

### Dependencies

See [capra_remote_controller dependencies](doc/dependencies.md)

### Usage

    You can simply launch the remote_controller with the command : 
    `roslaunch capra_remote_controller remote_controller.launch`

    You can check the published topics with :
    `rostopic list`

**Published topics**

    - \joy
    - \capra_remote_controller (the msg type of the topic is a cmd_vel)