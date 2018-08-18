#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

pub = rospy.Publisher('/capra/IMU', Imu, queue_size=10);

def ImuCallback(msg):
    # Take de message from odom and print the value we want
    # with 4 decimals
    #msg1 = msg.pose.pose.position.x
    #msg2 = msg.pose.pose.position.y
    #msg3 = msg.pose.pose.position.z
    msg1 = msg.orientation.x
    msg2 = msg.orientation.y
    msg3 = msg.orientation.z

    print 'position:'
    print ("    x : %.4f" % msg1)
    print ("    y : %.4f" % msg2)
    print ("    z : %.4f" % msg3)

    pub.publish(msg)
    
def listener():

    rospy.init_node('listener', anonymous=True)
    #listen to the odom node
    sub = rospy.Subscriber('/vectornav/IMU', Imu, ImuCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
		pass


