#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

if __name__ == "__main__":
    try:
        rospy.init_node('imu_zero')
        # publish (0,0,0,1)
        pub_imu_zero = rospy.Publisher("/imu/zero", Imu, queue_size=10)
        rate = rospy.Rate(10)
        zero = Imu()
        zero.header.frame_id = "odom"
        zero.orientation = Quaternion(0, 0, 0, 1)
        while not rospy.is_shutdown():
            pub_imu_zero.publish(zero)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass