Starting program: /home/nvidia/Code/capra_ws/devel/lib/takin_motors/motors_control __name:=motors_control __log:=/home/nvidia/.ros/log/4bed613c-719f-11e9-8640-00044b8d6521/motors_control-2.log
[tcsetpgrp failed in terminal_inferior: Inappropriate ioctl for device]
[tcsetpgrp failed in terminal_inferior: Inappropriate ioctl for device]
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/aarch64-linux-gnu/libthread_db.so.1".
[tcsetpgrp failed in terminal_inferior: Inappropriate ioctl for device]
[tcsetpgrp failed in terminal_inferior: Inappropriate ioctl for device]
[New Thread 0x7fb577e190 (LWP 8246)]
[New Thread 0x7fb4f7e190 (LWP 8247)]
[New Thread 0x7faffff190 (LWP 8248)]
[New Thread 0x7faf7ff190 (LWP 8253)]
using interface: can0
[New Thread 0x7faefff190 (LWP 8255)]
[New Thread 0x7fae7ff190 (LWP 8256)]
[New Thread 0x7fadfff190 (LWP 8257)]

Thread 1 "motors_control" received signal SIGSEGV, Segmentation fault.
0x0000000000000000 in ?? ()
(gdb) bt
#0  0x0000000000000000 in ?? ()
#1  0x0000000000456140 in ctre::phoenix::motorcontrol::can::BaseMotorController::Follow(ctre::phoenix::motorcontrol::IMotorController&, ctre::phoenix::motorcontrol::FollowerType) ()
#2  0x00000000004561c8 in ctre::phoenix::motorcontrol::can::BaseMotorController::Follow(ctre::phoenix::motorcontrol::IMotorController&) ()
#3  0x000000000044149c in setUpMotors(ros::NodeHandle&) ()
#4  0x00000000004418c8 in main ()

