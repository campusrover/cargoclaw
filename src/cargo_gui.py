#!/usr/bin/env python
import tkinter #this is the GUI I chose
from std_msgs.msg import String, Bool
import rospy
import rosnode
<<<<<<< Updated upstream
=======
import subprocess
import threading
import time
>>>>>>> Stashed changes


widgets=[]
screen1=[]
screen2=[]
screen3=[]

"""CargobotGUI"""

root=tkinter.Tk() #init window
root.wm_title("Control Panel") #set window name
root.geometry("500x600") #set size of window

message=" "
def ui_cb(msg): #recives string from robot node
    global message
    message=str(msg.data)
    updateMessages()
    updateTime()

def state_cb(msg):
    key_pub.publish("gh")

def arm_cb(msg):
    pass #TODO add message catchers for strings Vibhu sent in slack

#creates all the text variables for displaying the robot data
mode_text=tkinter.StringVar()
mode_text.set("Mode 1")

text1=tkinter.StringVar()
text1.set("Please Start Main Node")
text2=tkinter.StringVar()
text2.set(" ")
text3=tkinter.StringVar()
text3.set(" ")
text4=tkinter.StringVar()
text4.set(" ")
text5=tkinter.StringVar()
text5.set(" ")


#updates all the text for robot data
def updateMessages():
    # print(("/robot") in rosnode.get_node_names())
    if message!=" ":
        wordList=message.split("\n")
        text1.set(wordList[0])
        text2.set(wordList[1])
        text3.set(wordList[2])
        text4.set(wordList[3])
        text5.set(wordList[4])
    else:
        text1.set("Please Start Main Node")
        text2.set(" ")
        text3.set(" ")
        text4.set(" ")
        text5.set(" ")

def updateTime():
    """
    IDEA 1:
    make a heartbeat publisher that publishes in the main loop of the other node saying that it still exists,
    and if a certain amount of time has elapsed since GUI heartbeat_cb has run, then you display it stopped,
    and if there is a manual exit, send that through the heartbeat topic.

    IDEA 2:
    Google if there is a way in ROS to tell the status of another node.
    """
    pass

<<<<<<< Updated upstream
=======
def start_terminal_1(): #thread
    command1="roslaunch turtlebot3_gazebo turtlebot3_house.launch"
    subprocess.run(command1, shell=True,check=True)
def terminal2():
    command2="roslaunch turtlebot3_slam turtlebot3_slam.launch"
    subprocess.run(command2, shell=True, check=True)

>>>>>>> Stashed changes

def pack_test():
    for i in widgets:
        i.pack_forget()
    mode_text.set("Test Mode")

    mode_label.pack(side=tkinter.TOP)

    #creating labels to display robot data
    label1.pack(side=tkinter.TOP)
    label2.pack(side=tkinter.TOP)
    label3.pack(side=tkinter.TOP)
    label4.pack(side=tkinter.TOP)
    label5.pack(side=tkinter.TOP)

    #packing frames onto tkinter window
    frame1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    frame2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    frame3.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    bigframe.pack(side=tkinter.TOP,expand=True, fill=tkinter.BOTH)
    perm_frame.pack(side=tkinter.BOTTOM, fill=tkinter.BOTH)


    #packing buttons onto the frames in a percise order so it looks right
    SetGoal.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    forward.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    SetHome.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    left.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    hold.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    right.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    GoGoal.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    back.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    GoHome.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)



def pack1():
<<<<<<< Updated upstream
    for i in widgets:
        i.pack_forget()
    for i in screen1:
        print(i)
        i.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    forward.pack()
    hold.pack()
    left.pack()
    right.pack()
    back.pack()

def pack2():
    pass

def pack3():
    pass
=======
    #Run Subcommand
    # t1=threading.Thread(target=start_terminal_1)
    # t1.start()

    # time.sleep(2)

    t2=threading.Thread(target=terminal2)
    t2.start()
    #Pack and Unpack
    mode_text.set("Map Creation Mode")
    for i in widgets:
        i.pack_forget()
    for j in screen1:
        j.pack()

    forward.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    hold.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    left.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    right.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    back.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    frame1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    frame2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    frame3.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    bigframe.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)

    perm_frame.pack(side=tkinter.BOTTOM, fill=tkinter.BOTH)

def pack2():
    mode_text.set("Localization Mode")
    for i in widgets:
        i.pack_forget()
    for j in screen2:
        j.pack()

    #Widgets go here

    perm_frame.pack(side=tkinter.BOTTOM, fill=tkinter.BOTH)

def pack3():
    mode_text.set("Self Driving Mode")
    for i in widgets:
        i.pack_forget()
    for j in screen3:
        j.pack()

    #Widgets Here

    perm_frame.pack(side=tkinter.BOTTOM, fill=tkinter.BOTH)

>>>>>>> Stashed changes


#All functions that are called by pressing GUI buttons
def pubH():
    key_pub.publish("h")
def pubL():
    key_pub.publish("l")
def pubR():
    key_pub.publish("r")
def pubF():
    key_pub.publish("f")
<<<<<<< Updated upstream
=======
    print("Forward")
>>>>>>> Stashed changes
def pubB():
    key_pub.publish("b")

def pubSH():
    key_pub.publish("sh")
def pubGH():
    key_pub.publish("gh")
def pubSG():
    key_pub.publish("sg")
def pubGG():
    key_pub.publish("gg")


#initalizing rospy stuff 
rospy.init_node("GUI")
key_pub = rospy.Publisher('keys', String, queue_size=1) #publishes to the same topic as key_publisher.py
ui_sub = rospy.Subscriber('UI', String, ui_cb) #recives the information from the robot node in the UI topic
arm_subscriber=rospy.Subscriber("arm_status", String, arm_cb)


mode_label=tkinter.Label(root,textvariable=mode_text, width=500)
label1=tkinter.Label(root,textvariable=text1, width=500)
label2=tkinter.Label(root,textvariable=text2, width=500)
label3=tkinter.Label(root,textvariable=text3, width=500)
label4=tkinter.Label(root,textvariable=text4, width=500)
label5=tkinter.Label(root,textvariable=text5, width=500)





#frames to orgranize buttons
bigframe=tkinter.Frame(
    root
)
frame1=tkinter.Frame(
    bigframe
)
frame2=tkinter.Frame(
    bigframe
)
frame3=tkinter.Frame(
    bigframe
)

perm_frame=tkinter.Frame(
    root
)

#buttons to press to control the robot
forward=tkinter.Button(
    frame2,
    text="Forward",
    bg="light green",
    command=pubF
)

back=tkinter.Button(
    frame2,
    text="Back",
    bg="light green",
    command=pubB
)

left=tkinter.Button(
    frame1,
    text="Left",
    bg="light green",
    command=pubL
)

right=tkinter.Button(
    frame3,
    text="Right",
    bg="light green",
    command=pubR
)

hold=tkinter.Button(
    frame2,
    text="STOP",
    bg="red",
    command=pubH
)

SetHome=tkinter.Button(
    frame1,
    text="Set Home",
    bg="grey",
    command=pubSH
)

GoHome=tkinter.Button(
    frame1,
    text="Go Home",
    bg="grey",
    command=pubGH
)

SetGoal=tkinter.Button(
    frame3,
    text="Set Goal",
    bg="grey",
    command=pubSG
)

GoGoal=tkinter.Button( #an empty button which would have a different command but I ran out of time
    frame3,
    text="Go Goal",
    bg="grey",
    command=pubGG
)

# NextMode=tkinter.Button( #TODOs

# )

#Permanent Buttons
Mode1=tkinter.Button(
    perm_frame,
<<<<<<< Updated upstream
    text="Mode1",
=======
    text="Maping",
>>>>>>> Stashed changes
    bg="yellow",
    command=pack1
)

Mode2=tkinter.Button(
    perm_frame,
<<<<<<< Updated upstream
    text="Mode2",
=======
    text="Localizing",
>>>>>>> Stashed changes
    bg="yellow",
    command=pack2
)

Mode3=tkinter.Button(
    perm_frame,
<<<<<<< Updated upstream
    text="Mode3",
=======
    text="Driving",
>>>>>>> Stashed changes
    bg="yellow",
    command=pack3
)

widgets.append(mode_label)
widgets.append(label1)
widgets.append(label2)
widgets.append(label3)
widgets.append(label4)
widgets.append(label5)
for i in widgets:
    screen1.append(i)
    screen2.append(i)
    screen3.append(i)
widgets.append(SetHome)
widgets.append(GoHome)
widgets.append(SetGoal)
widgets.append(GoGoal)
widgets.append(forward)
widgets.append(back)
widgets.append(left)
widgets.append(right)
widgets.append(frame1)
widgets.append(frame2)
widgets.append(frame3)
widgets.append(hold)
widgets.append(perm_frame)
widgets.append(bigframe)


Mode1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
Mode2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
Mode3.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)

pack_test()


root.mainloop() #tktiner mainloop