#!/usr/bin/env python

import rospy
from capra_msgs.msg import SensorValue
from nav_msgs.msg import Odometry


#TODO : fonction qui ira chercher le CO2 level

# Reçoit les données du capteur CO2 et les publie
# Inclue dans le message publié la position et l'heure.
class CO2_sensor:

	def __init__(self):
		self.CO2_level = 400
		self.pose = None

	# Callback listening for odom/Odometry
	def read_pos(self, msg):
		self.pose = msg.pose

	# Main Loop
	def run(self):
		rospy.init_node('CO2_broadcast', anonymous=True)
		rospy.Subscriber('odom', Odometry, self.read_pos)
		pub = rospy.Publisher('CO2_level', SensorValue, queue_size=10)
		rate = rospy.Rate(1) # 1Hz a changer selon lebesoin
		while not rospy.is_shutdown():
			msg = SensorValue()
			msg.value = self.CO2_level 
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