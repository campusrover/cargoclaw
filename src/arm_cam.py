#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, random
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
import numpy as np

class ImageProcessor:
	
	def __init__(self):

		# Get image from the camera
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.process_image)
		
		# State the transportation robot (moving - False, halted - True)
		self.alien_state_sub = rospy.Subscriber("alien_state", Bool, self.set_state)
		self.alien_state = True
		
		# Send position of the cargo
		self.cargo_point_pub = rospy.Publisher("cargo_point", Point, queue_size=1)

		# Image processing
		self.bridge = cv_bridge.CvBridge()
		self.kernel = np.ones((7,7),np.uint8)
		
		# Pixel dimensions of image
		width_pixels = rospy.get_param("~width_pixels", 640)
		height_pixels = rospy.get_param("~height_pixels", 480)

		# Physical dimensions captured by image in meters
		width_phys = rospy.get_param("~width_phys", 0.228)
		height_phys = rospy.get_param("~height_phys", 0.181)

		# Convert pixels to physical dimensions
		self.width_ratio = width_phys / width_pixels 
		self.height_ratio = height_phys / height_pixels 

		# z distance from arm at rest position to the box
		self.arm_z = rospy.get_param("~arm_z", -0.06)

	def set_state(self, msg):
		self.alien_state = msg.data

	def process_image(self, msg):
		# Only process if the transportation robot is halted
		if (self.alien_state):
			
			# Get image
			image = self.bridge.imgmsg_to_cv2(msg)

			# Yellow mask
			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

			lower_yellow = np.array([ 25, 0, 230 ])
			upper_yellow = np.array([ 255, 255, 255 ])
			mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
			
			# Remove noise from mask
			mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

			# Perform filtering
			masked = cv2.bitwise_and(image, image, mask=mask)
			masked = cv2.cvtColor(masked, cv2.COLOR_HSV2BGR)
			masked = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)

			# Get coordinates of the box
			x = 4
			y = 4	# Invalid by default. If not changed by computed centroid, forces robot to reposition

			# Compute the centroid
			M = cv2.moments(masked)

			# If a centroid can be found, the box is in view
			if M['m00'] != 0:

				# Compute centroid
				x_pixels = int(M['m10']/M['m00'])
				y_pixels = int(M['m01']/M['m00'])

				print(f"x pix: {x_pixels} y pix: {y_pixels}")

				# Compute physical x/y coordinates
				x = x_pixels*self.width_ratio
				y = y_pixels*self.height_ratio

				print(f"x: {x} y: {y}")

			# Publish the position of the cargo box
			self.cargo_point_pub.publish(Point(x, y, self.arm_z))

			cv2.imshow("image", image)
#			cv2.imshow("hsv", hsv)
			cv2.imshow("masked", masked)

			cv2.waitKey(3)
			
			#temp testing and setting state to false
#			self.alien_state = False

rospy.init_node("arm_cam")
image_processor = ImageProcessor()
rospy.spin()