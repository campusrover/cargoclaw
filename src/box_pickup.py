#!/usr/bin/env python3
"""
A basic ROS command line controller for the Interbotix PX100 Arm. 
Run the program, read the README or code, to find more details about the commands
"""

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32
import time

class SendCommand():

    def __init__(self):
        # print initial title and notes 
        print("THE COMMAND LINE PX100 ARM CONTROLLER!")
        print("NOTE: The y value is actually moving the waist of the arm which uses a different command than the x and z values which sets the joint position.\n" 
                "The range is (-3.1, 3.1). If you enter the same value twice, the waist will not move because it is already at the set coordinate.")

        # initialize variables 
        self.x = 0 
        self.y = 4
        self.z = 0
        self.exit = "exit"
        self.home = "home"
        self.sleep = "sleep"
        self.open_gripper = "og"
        self.close_gripper = "cg"
        self.open = "open"
        self.close = "close"

        #Subscribers
        self.point_sub = rospy.Subscriber("cargo_point", Point, self.cargo_point_cb)
        
        # set up publishers for all the different messages that the controller will receive
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
        self.pose_publisher = rospy.Publisher("/arm_control/pose", Point, queue_size=1)
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.exit_publisher = rospy.Publisher("/arm_control/exit", Bool, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm/time", Float32, queue_size=1)
        self.arm_status_publisher = rospy.Publisher("/arm_status", String, queue_size=1)
        
        self.publisher()

        

    #base values when robot is perfect positin in front of arm p 0 .04 -.06
    def publisher(self):
        while True: 
            if self.is_valid_coordinate(self.y):
                self.arm_status_publisher.publish("grabcube")
                self.home_publisher.publish(True)
                time.sleep(1.5)
                self.gripper_publisher.publish(self.open)
                time.sleep(1.5)
                self.point_publisher.publish(Point(self.x, self.y, self.z))
                time.sleep(1.5)
                self.gripper_publisher.publish(self.close)
                time.sleep(1.5)
                self.home_publisher.publish(True)
                time.sleep(1.5)
                self.point_publisher.publish(Point(+0, -1, -0.12))
                time.sleep(1.5)
                self.gripper_publisher.publish(self.open)
                time.sleep(1.5)
                self.point_publisher.publish(Point(0, -1, 0.12))
                time.sleep(1.5)
                self.sleep_publisher.publish(True)
                self.arm_status_publisher.publish("resting")
    
    # def is_dig(self,n):
    #     try:
    #         float(n)
    #         return True
    #     except ValueError:
    #         return  False

    def is_valid_coordinate(self, y_value):
        if (self.y > 3.1 or self.y < -3.1):
            self.arm_status_publisher.publish("invalid")
            rospy.loginfo("The box's coordinates are invalid, need to reposition robot")
            return False
        self.arm_status_publisher.publish("valid")
        return True

    def cargo_point_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        print("coordinates " + self.x + self.y + self. z)


if __name__=='__main__':
    rospy.init_node("box_pickup")
    try:
        SendCommand()
    except rospy.ROSInterruptException:
        pass
