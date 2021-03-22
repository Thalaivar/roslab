#!/usr/bin/env python

import math
import yaml
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from image_processing import image_processing

from cv_bridge import CvBridge

class Explore:
	def __init__(self, n_joints=3, update_freq=100, rps=2 * math.pi / 60):
		rospy.init_node("joint_positions_node", anonymous=True)

		self.joint_names = [f"joint{x}" for x in range(n_joints)]
		self.pubs = {x: rospy.Publisher(
						f"/rrbot/{x}_position_controller/command", 
						Float64, 
						queue_size=10) for x in self.joint_names}
		
		self.sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.process_raw_image)
		
		self.mask_pub = rospy.Publisher("/rrbot/bbox_image", Image, queue_size=10)

		self.rate = rospy.Rate(update_freq)
		self.rps = rps / update_freq

		self.bridge = CvBridge()
		self.detected = False
		self.detect_thresh = 60000

	def process_raw_image(self, raw_image: Image):
		image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding="passthrough")
		area, _, masked_image = image_processing(image)

		if area > 0:
			self.mask_pub.publish(self.bridge.cv2_to_imgmsg(masked_image, encoding="passthrough"))
		else:
			self.mask_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="passthrough"))
		if area > self.detect_thresh:
			self.detected = True
		
	def hold_at_angle(self, joint_angles=[math.pi / 2, math.pi / 4, 0]):
		curr_ang = {name: x for name, x in zip(self.joint_names, joint_angles)}
		while not rospy.is_shutdown():
			for name, pub in self.pubs.items():
				pub.publish(curr_ang[name])

			self.rate.sleep()

	def exploratory(self):
		"""
			The strategy followed to explore and find the object is:
				1. Rotate base joint from 0 -> 2*pi
				2. Fix joint1's angle at pi/4
				3. For every base joint, rotate joint2 from -pi/4 -> pi/2
			
			The assumption is made that the object is on the ground. If this
			assumption holds, then we should be able to find the object with
			this strategy
		"""
		curr_ang = {name: 0 for name in self.joint_names}
		curr_ang["joint1"] = math.pi / 4
		while not rospy.is_shutdown() and not self.detected:
			# rotate base joint for 2*pi radians
			curr_ang["joint0"] %= 2 * math.pi	
			self.pubs["joint0"].publish(curr_ang["joint0"])

			# fix first joint at specified angle
			self.pubs["joint1"].publish(curr_ang["joint1"])

			# rotate first joint between 0 and pi / 2
			curr_ang["joint2"] = 0
			while curr_ang["joint2"] <= math.pi / 4 and not self.detected:
				self.pubs["joint2"].publish(curr_ang["joint2"])

				curr_ang["joint2"] += self.rps
				self.rate.sleep()
			
			if not self.detected:
				curr_ang["joint0"] += (math.pi / 4)
			self.rate.sleep()

		if self.detected:
			hold_angles = [x for _, x in curr_ang.items()]
			rospy.loginfo(f"Object detected! Holding at: {[round(x * 180 / math.pi, 2) for x in hold_angles]}")
			self.hold_at_angle(hold_angles)

if __name__ == "__main__":
	try:
		exp = Explore()
		exp.exploratory()
	except rospy.ROSInterruptException: 
		pass