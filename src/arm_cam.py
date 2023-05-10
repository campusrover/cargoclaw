#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class ImageProcessor:
	
	def __init__(self):

		# Get image from the camera
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.process_image)
		
		# State the transportation robot (moving - False, halted - True)
		self.alien_state_sub = rospy.Subscriber("alien_state", Bool, self.set_state)
		self.alien_state = True
		
		# Send position of the cargo
		self.cargo_point_pub = rospy.Publisher("cargo_point", Point, queue_size=10)

		# Image processing
		self.bridge = cv_bridge.CvBridge()
		self.kernel = numpy.ones((7,7), numpy.uint8)
		
		# Pixel dimensions of image
		width_pixels = rospy.get_param("~width_pixels", 640)
		height_pixels = rospy.get_param("~height_pixels", 480)

		# Physical dimensions captured by image in meters
		width_phys = rospy.get_param("~width_phys", 0.228)
		height_phys = rospy.get_param("~height_phys", 0.181)

		# Convert pixels to physical dimensions
		self.width_ratio = width_phys / width_pixels 
		self.height_ratio = height_phys / height_pixels 

		# z distance from arm at "home" position to the box
		self.arm_z = rospy.get_param("~arm_z", -0.06)

	def set_state(self, msg):
		self.alien_state = msg.data

	def process_image(self, msg):
		# Only process if the transportation robot is halted
		if (self.alien_state):
			
			# Get image
			image = self.bridge.imgmsg_to_cv2(msg)

			# Color mask
			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

			lower_mask = numpy.array([ 80, 0, 230 ])
			upper_mask = numpy.array([ 90, 255, 255 ])
			mask = cv2.inRange(hsv, lower_mask, upper_mask)
			
			# Remove noise from mask
			mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

			# Perform filtering
			masked = cv2.bitwise_and(image, image, mask=mask)
			masked = cv2.cvtColor(masked, cv2.COLOR_HSV2BGR)
			masked = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)

			# Compute the centroid
			M = cv2.moments(masked)

			# If a centroid can be found, the cargo is in view
			if M['m00'] != 0:

				# Compute centroid
				x_pixels = int(M['m10']/M['m00'])
				y_pixels = int(M['m01']/M['m00'])

				# Compute physical x/y coordinates
				x = x_pixels*self.width_ratio
				y = y_pixels*self.height_ratio

				print(x, y)

				# Publish the position of the cargo box
				self.cargo_point_pub.publish(Point(x, y, self.arm_z))

			else:
				# Cargo is not in view, publish an invalid z
				self.cargo_point_pub.publish(Point(0, 0, 10))

			cv2.waitKey(3)

rospy.init_node("arm_cam")
image_processor = ImageProcessor()
rospy.spin()