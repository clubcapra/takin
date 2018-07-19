#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def main():
        
    rospy.init_node("cmd_vel_simulator")

    pub = rospy.Publisher("/capra_simulation/cmd_vel", Twist, queue_size=10)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 1
        # msg.angular.z = 0.1

        pub.publish(msg)

if __name__ == "__main__":
    main()