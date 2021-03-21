#!/usr/bin/env python

import math
import yaml
import rospy

from std_msgs.msg import Float64

def get_joint_angle(x, n):
	return math.pi * math.sin(x / n)

def rrbot_exploratory_publisher(n=100):
	rospy.init_node("joint_positions_node", anonymous=True)
	
	joint_names = ["joint0", "joint1", "joint2"]
	pubs = {x: rospy.Publisher(
						f"/rrbot/{x}_position_controller/command", 
						Float64, 
						queue_size=10) for x in joint_names}
	rate = rospy.Rate(100)

	"""
		The strategy followed to explore and find the object is:
			1. Rotate base joint from 0 -> 2*pi
			2. Fix joint1's angle at 0
			3. For every base joint, rotate joint2 from -pi/2 -> pi/2
		
		The assumption is made that the object is on the ground. If this
		assumption holds, then we should be able to find the object with
		this strategy
	"""
	curr_ang = {name: 0 for name in joint_names}
	curr_ang["joint1"] = 0
	curr_ang["joint2"] = -math.pi * n / 8
	while not rospy.is_shutdown():
		# rotate base joint for 2*pi radians
		# curr_ang["joint0"] %= (2 * math.pi * n)	
		pubs["joint0"].publish(get_joint_angle(curr_ang["joint0"], n))

		# fix first joint at specified angle
		pubs["joint1"].publish(get_joint_angle(curr_ang["joint1"], n))

		# rotate first joint between -pi / 2 and pi / 2
		# curr_ang["joint2"] = -math.pi * n / 6
		# while curr_ang["joint2"] <= math.pi * n / 6:
		# 	pubs["joint2"].publish(get_joint_angle(curr_ang["joint2"], n))
		# 	curr_ang["joint2"] += 1
		# 	rate.sleep()
		pubs["joint2"].publish(get_joint_angle(curr_ang["joint2"], n))
		# curr_ang["joint0"] += 1
		rate.sleep()

if __name__ == "__main__":
	try: rrbot_exploratory_publisher(n=100)
	except rospy.ROSInterruptException: pass