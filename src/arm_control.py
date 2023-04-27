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
        self.y = 0
        self.z = 0
        self.exit = "exit"
        self.home = "home"
        self.sleep = "sleep"
        self.open_gripper = "og"
        self.close_gripper = "cg"
        self.open = "open"
        self.close = "close"
        
        # set up publishers for all the different messages that the controller will receive
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
        self.pose_publisher = rospy.Publisher("/arm_control/pose", Point, queue_size=1)
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.exit_publisher = rospy.Publisher("/arm_control/exit", Bool, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm/time", Float32, queue_size=1)
        
        self.publisher()

        self.point_sub = rospy.Subscriber("/cargo_point", Point, queue_size=1)


    

    # def print_commands(self):
    #     print(">-----------------COMMANDS----------------------<")
    #     print("Enter a point (x,y,z): p 0 0 0")
    #     print("Enter ee pose matrix (x,y,z): m 0 0 0")
    #     print("Set trajectory time: t 1.0")
    #     print("Go to home position: home")
    #     print("Go to sleep position: sleep")
    #     print("Open gripper: og")
    #     print("Close gripper: cg")
    #     print("Exit: exit")
    #     print(">-----------------------------------------------<")

        

    def publisher(self):
        while True: 
            # move by point amount
            elif user_input[0] == "p":
                user_input = user_input.split(" ")
                can_send = self.check_input(user_input)
                if can_send:
                    self.point_publisher.publish(Point(self.x, self.y, self.z))
            # set ee pose components
            elif user_input[0] == "m":
                user_input = user_input.split(" ")
                can_send = self.check_input(user_input)
                if can_send:
                    self.pose_publisher.publish(Point(self.x, self.y, self.z))
            elif user_input[0] == "t":
                user_input = user_input.split(" ")
                if self.is_dig(user_input[1]):
                    self.time_publisher.publish(float(user_input[1]))
            elif user_input[0] == "z":
                self.home_publisher.publish(True)
                time.sleep(1.5)
                self.gripper_publisher.publish(self.open)
                time.sleep(1.5)
                self.point_publisher.publish(Point(0, 1, -0.14))
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
    
    def is_dig(self,n):
        try:
            float(n)
            return True
        except ValueError:
            return  False
    

if __name__=='__main__':
    rospy.init_node("send_message")
    try:
        SendCommand()
    except rospy.ROSInterruptException:
        pass
