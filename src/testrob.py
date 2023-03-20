#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import os
import signal

"""Good standard starting code for testing a robot"""

class Robot:
    def __init__(self):
        self.OGSpeed=0.2 #original speed of robot
        self.speed=self.OGSpeed #used speed of robot
        self.turnspeed=1 #rad/sec
        self.vel=0#velocity storing variable
        self.x=0 #x position
        self.y=0 #y position
        self.yaw=0 #yaw  
        
        self.firstZig=False #is currently zig zagging
        self.part="" #part of zigzag
        self.ogPoint=(0,0) #original location
        self.ogYaw=0 #original Yaw
        self.dancing=False #checks if currently is dancing

        self.state="H"
        self.last_key_press_time=rospy.Time.now()
    
    def clear(self): #clears output
        os.system('clear')

    def checkDist(self,x1,y1,x2,y2): #Performs a simple calculation to find the distance between the current x and y position and the start position
        x3=(x1-x2)**2
        y3=(y1-y2)**2
        return math.sqrt(x3+y3)

    # print the state of the robot
    def print_state(self):
        self.clear()
        # calculate time since last key stroke
        time_since = rospy.Time.now() - self.last_key_press_time
    
        #print for terminal
        print("---")
        print("Velocity: " + str(self.vel))
        print("Position: ("+ str(self.x)+","+ str(self.y)+")")
        print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))
        print("---")
    
        #return as single string for GUI
        return("---\nSTATE: " + self.state+"\nVelocity: " + str(self.vel)+"\nPosition: ( x= "+ str(self.x)+", y="+ str(self.y)+")\nSECS SINCE LAST BUTTON PRESS: " + str(time_since.secs)+"\n---")
    
    


    def moveForward(self): #sets movement to forwards
        t.linear.x = self.speed
    def moveBack(self): #Sets movement back
        t.linear.x = -self.speed
    def moveHold(self): #stops movement
        t.linear.x=0.0

    def turnLeft(self): #sets turn left
        t.angular.z = self.turnspeed
    def turnRight(self): #sets turn left
        t.angular.z = -self.turnspeed
    def turnHold(self): #stops turn
        t.angular.z = 0.0

    def checkMove(self):
        if self.state=="ff" or self.state=="bb":
            self.speed=self.OGSpeed+0.2
        else:
            self.speed=self.OGSpeed

        # if state=="s":
        #     turnspeed-=0.001
        # else:
        #     speed=0.2
        #     turnspeed=0.5

        if self.state=="f":
            self.moveForward()
        elif self.state=="ff":
            self.moveForward()
        elif self.state=="b":
            self.moveBack()
        elif self.state=="bb":
            self.moveBack()
        elif self.state=="l":
            self.turnLeft()
        elif self.state=="r":
            self.turnRight()
        
        
        elif self.state=="s":
            if (self.yaw<0 and self.yaw > -(math.pi/2)):
                self.speed=0.4
            else:
                self.speed=0.2
            self.turnLeft()
            self.moveForward()
        elif self.state=="z":
            if self.firstZig==False:
                self.firstZig=True
                self.zig=False
                self.zag=True
                self.ogYaw=self.yaw
                self.part='r'
            else:
                if self.part=='l':
                    self.moveHold()
                    self.turnLeft()
                    if (self.yaw-self.ogYaw)>math.pi/6:
                        self.part='fr'
                        self.ogPoint=(self.x,self.y)
                elif self.part=='r':
                    self.moveHold()
                    self.turnRight()
                    if (self.yaw-self.ogYaw)<math.pi/6:
                        self.part='fl'
                        self.ogPoint=(self.x,self.y)
                else:
                    self.turnHold()
                    self.moveForward()
                    if self.checkDist(self.x, self.y, self.ogPoint[0], self.ogPoint[1])>1:
                        if self.part=='fl':
                            self.part='l'
                        else:
                            self.part='r'
        elif self.state=="d":
            if not self.dancing:
                self.dancing=True
                self.part="r"
                self.ogYaw=self.yaw
            if self.part=="r":
                if (self.yaw-self.ogYaw)<math.pi/6:
                    self.part="l"
                else:
                    self.turnRight()
            elif self.part=="l": 
                if (self.yaw-self.ogYaw)>math.pi/6:
                    self.part="r"
                else:
                    self.turnLeft()

        elif self.state=="h":
            self.moveHold()
            self.turnHold()
        else:
            self.moveHold()
            self.turnHold()
        


        if self.firstZig and self.state!="z":
            self.firstZig=False
        if self.dancing and self.state!="d":
            self.dancing=False

    def shutdown(self,sig, stackframe):
        print("Exit because of ^c")
        self.moveHold()
        self.turnHold()
        sys.exit(0)


def vel_cb(msg):#takes predicted velocity from the cmd_vel
    rob.vel= msg.linear.x

    # Scan Callback
def scan_cb(msg):
    pass
    # print(min(msg.ranges))
    # if min(msg.ranges)<0.1:
    #     rob.state='h'

#Key press callback
def key_cb(msg):
    if rob.state==msg.data and (rob.state=="f" or rob.state=="b"): #keeps track of if its a double input or not
        rob.state="" + msg.data+""+msg.data
    else:
        rob.state = msg.data
    rob.last_key_press_time = rospy.Time.now()

# odom data
def odom_cb(msg):
    rob.x=msg.pose.pose.position.x
    rob.y=msg.pose.pose.position.y
    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, rob.yaw) = euler_from_quaternion(quarternion)

#Controls Shutdown
def shutdown(sig, stackframe):
    print("\nControlled Exit Due To ^C")
    rob.moveHold()
    rob.turnHold()
    cmd_vel_pub.publish(t)
    # cmd_vel_pub.publish(t)
    sys.exit(0)


# init node
rospy.init_node('robot')
rob=Robot()

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb) 
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, vel_cb)
ui_pub=rospy.Publisher("UI", String, queue_size=10) #publishes to my own topic with the string for the GUI

# start in state halted and grab the current time
rob.last_key_press_time = rospy.Time.now()

# set rate
rate = rospy.Rate(10)

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

   # print out the current state and time since last key press

   # publish cmd_vel from here 
    t = Twist()

    rob.checkMove()

    cmd_vel_pub.publish(t)
    ui_pub.publish(rob.print_state())

    signal.signal(signal.SIGINT, shutdown)

   # run at 10hz
    rate.sleep()
    