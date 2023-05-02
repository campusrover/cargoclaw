#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import os
import signal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

"""Code for moving Cargo Bot"""

class Robot:
    def __init__(self):
        self.OGSpeed=0.2 #original speed of robot
        self.speed=self.OGSpeed #used speed of robot
        self.turnspeed=1 #rad/sec
        self.x=0 #x position
        self.y=0 #y position
        self.yaw=0 #yaw  
        self.quat=[]
        self.waypoints=[[],[]]

        self.state="H"
        self.last_key_press_time=rospy.Time.now()
    
    def setHome(self):
        self.waypoints[0]=self.createWaypoint()
        print(self.waypoints[0])
    def setGoal(self):
        self.waypoints[1]=self.createWaypoint()
        print(self.waypoints[0])

    def createWaypoint(self):
        return [[self.x,self.y,0],[0,0,self.quat[2],self.quat[3]]]

#Move set
    def goal_pose(self,pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def clear(self): #clears output
        os.system('clear')

    def checkDist(self,x1,y1,x2,y2): #Performs a simple calculation to find the distance between the current x and y position and the start position
        x3=(x1-x2)**2
        y3=(y1-y2)**2
        return math.sqrt(x3+y3)

    # print the state of the robot
    def print_state(self):
        # print(self.state)
        # self.clear()
        # # calculate time since last key stroke
        time_since = rospy.Time.now() - self.last_key_press_time
    
        # #print for terminal
        # print("---")
        # # print("Velocity: " + str(self.vel))
        # print("Position: ("+ str(self.x)+","+ str(self.y)+")")
        # print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))
        # print("---")
    
        # #return as single string for GUI
        return("---\nSTATE: " + self.state+"\nPosition: ( x= "+ str(self.x)+", y="+ str(self.y)+")\nSECS SINCE LAST BUTTON PRESS: " + str(time_since.secs)+"\n---")
    
    


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
        
        elif self.state=="sh":
            self.setHome()
            self.state="h"
        elif self.state=="gh":
            try:
                goal = self.goal_pose(self.waypoints[0])
                print("Going for goal: ", goal)
                client.send_goal(goal)
                # client.wait_for_result()
                self.state="Auto-Move"

            except:
                print("No Waypoint set")
                self.state='h'
        elif self.state=="sg":
            self.setGoal()
            self.state="h"
        elif self.state=="gg":
            try:
                goal = self.goal_pose(self.waypoints[1])
                print("Going for goal: ", goal)
                client.send_goal(goal)
                # client.wait_for_result()
            except:
                print("No Waypoint set")
            self.state="Auto-Move"
        
        elif self.state=="Auto-Move":
            pass

        elif self.state=="h":
            self.moveHold()
            self.turnHold()
        else:
            self.moveHold()
            self.turnHold()
        

    def shutdown(self,sig, stackframe):
        print("Exit because of ^c")
        self.moveHold()
        self.turnHold()
        sys.exit(0)


#Key press callback
def key_cb(msg):
    if rob.state==msg.data and (rob.state=="f" or rob.state=="b"): #keeps track of if its a double input or not
        rob.state="" + msg.data+""+msg.data
    elif rob.state=="Auto-Move":
        if (msg.data=="h"):
            goal = rob.goal_pose(rob.createWaypoint())
            print("Going for goal: ", goal)
            client.send_goal(goal)
            print("SHOULD HAVE STOPPED")
            rob.state="h"
        """Make a waypoint at robots current location, set that waypoint to the goal"""

    else:
        rob.state = msg.data
    rob.last_key_press_time = rospy.Time.now()

# odom data
def odom_cb(msg):
    rob.x=msg.pose.pose.position.x
    rob.y=msg.pose.pose.position.y
    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, rob.yaw) = euler_from_quaternion(quarternion)
    rob.quat=quarternion
    # print(rob.quat)
    

zero_flag=0
def vel_cb(msg):
    global zero_flag
    if rob.state=="Auto-Move":
        if (msg.linear.x==0)and (msg.angular.z==0):
            if zero_flag==10:
                print(str(msg.linear.x)+". FINISHED")
                rob.state="h"
                if(rob.state=="gg"):
                    alien_pub.publish(True)
                zero_flag=0
            else:
                zero_flag+=1
        else:
            zero_flag=0

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


# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb) 
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
ui_pub=rospy.Publisher("UI", String, queue_size=10) #publishes to my own topic with the string for the GUI
cmd_vel_sub=rospy.Subscriber("cmd_vel", Twist, vel_cb)
alien_pub=rospy.Publisher("alien_state",Bool, queue_size=1)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

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
    