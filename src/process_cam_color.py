#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, random
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.process_image)
        self.pose_publisher = rospy.Publisher("/arm_control/pose", Point, queue_size=1)



        self.kernel = np.ones((7,7),np.uint8)
        
        # pixel dimensions of image
        width_pixels = rospy.get_param("~width_pixels", 640)
        height_pixels = rospy.get_param("~height_pixels", 480)

        # physical dimensions captured by image in meters
        width_phys = rospy.get_param("~width_phys", 0.228)
        height_phys = rospy.get_param("~height_phys", 0.181)

        # convert pixels to physical dimensions
        self.width_ratio = width_phys / width_pixels 
        self.height_ratio = height_phys / height_pixels 

        # z distance from arm at home position to the box
        self.arm_z = rospy.get_param("~arm_z", 0.1)

    def process_image(self, msg):

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        cv2.imshow("image", image)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 40, 70, 70 ])
        upper_yellow = np.array([ 100, 1000, 1000 ])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # Remove noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        masked = cv2.bitwise_and(image, image, mask=mask)
        masked2 = cv2.cvtColor(masked, cv2.COLOR_HSV2BGR)
        masked3 = cv2.cvtColor(masked2, cv2.COLOR_RGB2GRAY)

        cv2.imshow("hsv", hsv)
        cv2.imshow("masked3", masked3)

        # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(masked3)
        print("M00 %d" % (M['m00']))

        # If a centroid can be found, the box is in view
        if M['m00'] != 0:

            # Compute centroid
            x_pixels = int(M['m10']/M['m00']) + 100
            y_pixels = int(M['m01']/M['m00'])

            print(f"x pix: {x_pixels} y pix: {y_pixels}")

            # Compute physical x/y coordinates
            x = x_pixels*self.width_ratio
            y = y_pixels*self.height_ratio

            print(f"x: {x} y: {y}")

            # Send to the arm
#            self.pose_publisher.publish(Point(x, y, self.arm_z))

        cv2.waitKey(3)

rospy.init_node("arm_cam")
image_processor = ImageProcessor()
rospy.spin()