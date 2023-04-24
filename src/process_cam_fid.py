#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, random
from fiducial_msgs.msg import FiducialArray
from geometry_msgs.msg import Twist, Point

class ImageProcessor:
    def __init__(self):
        self.fid_sub - rospy.Subscriber("/fiducial_vertices", FiducialArray, self.process_image)

        self.offset_x = rospy.get_param("~offset_x", 0.5)
        self.offset_y = rospy.get_param("~offset_y", 0.5)
        self.offset_z = rospy.get_param("~offset_z", 0.5)

        self.pose_publisher = rospy.Publisher("/arm_control/pose", Point, queue_size=1)

    def process_image(self, msg):

        fiducial =  msg.fiducials
        
        # no fiducial in view
        if len(fiducial) < 1:
            return

        

rospy.init_node("arm_cam")
image_processor = ImageProcessor()
rospy.spin()