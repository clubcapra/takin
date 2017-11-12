#!/usr/bin/env python

import rospy
from sensor_CO2.msg import CO2_level
from nav_msgs.msg import Odometry


#TODO : fonction qui ira chercher le CO2 level

class CO2_sensor:

	def __init__(self):
		self.CO2_level = 400
		self.pose = None

	def read_pos(self, msg):
		self.pose = msg.pose

	def run(self):
		rospy.init_node('CO2_broadcast', anonymous=True)
		rospy.Subscriber('odom', Odometry, self.read_pos)
		pub = rospy.Publisher('CO2_level', CO2_level, queue_size=10)
		rate = rospy.Rate(1) #1Hz a changer selon lebesoin
		while not rospy.is_shutdown():
			msg = CO2_level()
			msg.CO2_level = self.CO2_level 
			msg.time_ref = rospy.Time.now()
			msg.pose = self.pose
			rospy.loginfo(msg)
			pub.publish(msg)
			rate.sleep()


if __name__ == '__main__':
	try:
		sensor = CO2_sensor()
		sensor.run()
	except rospy.ROSInterruptException:
		pass