#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, random
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class ImageProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.process_image)
        self.pose_publisher = rospy.Publisher("/arm_control/pose", Point, queue_size=1)

    def process_image(self, msg):

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 40, 0, 0])
        upper_yellow = numpy.array([ 120, 255, 255])
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

        # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(restricted)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        # If a centroid can be found, the box is in view
        if M['m00'] != 0:

            # Compute the x/y coordinates
            x = int(M['m10']/M['m00']) + 100
            y = int(M['m01']/M['m00'])
            cv2.circle(image, (x, y), 20, (0,0,255), -1)

            # Compute the z coordinate
            

            # Send to the arm
            # self.pose_publisher.publish(Point(x, y, z))
                
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node("arm_cam")
image_processor = ImageProcessor()
rospy.spin()