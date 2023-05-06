#!/usr/bin/env python3
"""
A basic ROS command line controller for the Interbotix PX100 Arm. 
Run the program, read the README or code, to find more details about the commands
"""

import rospy, numpy, time
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32

class SendCommand():

    def __init__(self):
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
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.exit_publisher = rospy.Publisher("/arm_control/exit", Bool, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm/time", Float32, queue_size=1)
        self.arm_status_publisher = rospy.Publisher("/arm_status", String, queue_size=1)
        
        self.publisher()

    # Transform polar coordinates into arm command coordinates
    # TODO: FIGURE OUT TRANS_X
    def trans_x(self, x):
        return 0
    def trans_y(self, y):
        return -1.37406*y+0.94325

    #base values when robot is perfect position in front of arm p 0 .04 -.06
    def publisher(self):
        if self.is_valid_coordinate(self.y):
            self.arm_status_publisher.publish("grabcube")
            self.home_publisher.publish(True)
            time.sleep(1.5)
            self.gripper_publisher.publish(self.open)
            time.sleep(1.5)
            print(self.x, self.y)
            self.point_publisher.publish(Point(self.x, self.y, self.z))
            time.sleep(1.5)
            self.gripper_publisher.publish(self.close)
            time.sleep(1.5)
            self.home_publisher.publish(True)
            time.sleep(1.5)
            self.drop_cargo()

    def drop_cargo(self):
            self.point_publisher.publish(Point(0, -1, -0.12))
            time.sleep(1.5)
            self.point_publisher.publish(Point(-.15, -1, -0.12))
            time.sleep(1.5)
            self.gripper_publisher.publish(self.open)
            time.sleep(1.5)
            self.point_publisher.publish(Point(0, -1, 0.12))
            time.sleep(1.5)
            self.home_publisher.publish(True)
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
        x = msg.x
        y = msg.y
        self.z = msg.z      # No conversion necessary

        # Convert to polar
        r = numpy.sqrt(x**2 + y**2)
        t = numpy.arctan2(y, x)

        # Transform polar coordinates to arm command coordinates
        self.x = self.trans_x(r)
        self.y = self.trans_y(t)

        # Adjust for orientation of cube
        if t > 0.87:
            if self.y > 0:
                self.y += .05
            else:
                self.y -= 0.05

        self.publisher()

'''
if __name__=='__main__':
    rospy.init_node("box_pickup")
    try:
        SendCommand()
    except rospy.ROSInterruptException:
        pass
'''

rospy.init_node("box_pickup")
commander = SendCommand()
rospy.spin()