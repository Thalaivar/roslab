#!/usr/bin/env python

import math
import yaml
import rospy

from std_msgs.msg import Float64

def rrbot_exploratory_publisher(update_rate=100, rps=2 * math.pi / 60):
	rospy.init_node("joint_positions_node", anonymous=True)
	
	joint_names = ["joint0", "joint1", "joint2"]
	pubs = {x: rospy.Publisher(
						f"/rrbot/{x}_position_controller/command", 
						Float64, 
						queue_size=10) for x in joint_names}
	rate = rospy.Rate(update_rate)
	rps = rps / update_rate

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
	curr_ang["joint1"] = math.pi / 4
	while not rospy.is_shutdown():
		# rotate base joint for 2*pi radians
		curr_ang["joint0"] %= 2 * math.pi	
		pubs["joint0"].publish(curr_ang["joint0"])

		# fix first joint at specified angle
		pubs["joint1"].publish(curr_ang["joint1"])

		# rotate first joint between pi / 4 and 3 * pi / 4
		curr_ang["joint2"] = math.pi / 4
		while curr_ang["joint2"] <= math.pi * n / 6:
			pubs["joint2"].publish(curr_ang["joint2"])
			curr_ang["joint2"] += rps
			rate.sleep()
		
		curr_ang["joint0"] += rps
		rate.sleep()

if __name__ == "__main__":
	try: rrbot_exploratory_publisher(n=100)
	except rospy.ROSInterruptException: pass