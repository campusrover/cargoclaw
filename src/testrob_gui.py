#!/usr/bin/env python
import tkinter #this is the GUI I chose
from std_msgs.msg import String
import rospy
import rosnode


"""MAKE THE ROBOT DANCE GUI"""

root=tkinter.Tk() #init window
root.wm_title("Control Panel") #set window name
root.geometry("500x500") #set size of window

message=" "
def ui_cb(msg): #recives string from robot node
    global message
    message=str(msg.data)
    updateMessages()
    updateTime()

#creates all the text variables for displaying the robot data
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
text6=tkinter.StringVar()
text6.set(" ")

#updates all the text for robot data
def updateMessages():
    print(("/robot") in rosnode.get_node_names())
    if message!=" ":
        wordList=message.split("\n")
        text1.set(wordList[0])
        text2.set(wordList[1])
        text3.set(wordList[2])
        text4.set(wordList[3])
        text5.set(wordList[4])
        text6.set(wordList[5])
    else:
        text1.set("Please Start Main Node")
        text2.set(" ")
        text3.set(" ")
        text4.set(" ")
        text5.set(" ")
        text6.set(" ")

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

#All functions that are called by pressing GUI buttons
def pubH():
    key_pub.publish("h")
def pubL():
    key_pub.publish("l")
def pubR():
    key_pub.publish("r")
def pubF():
    key_pub.publish("f")
def pubB():
    key_pub.publish("b")
def pubS():
    key_pub.publish("s")
def pubZ():
    key_pub.publish("z")
def pubD():
    key_pub.publish("d")



#initalizing rospy stuff 
rospy.init_node("tkinter")
key_pub = rospy.Publisher('keys', String, queue_size=1) #publishes to the same topic as key_publisher.py
ui_sub = rospy.Subscriber('UI', String, ui_cb) #recives the information from the robot node in the UI topic


#creating labels to display robot data
label1=tkinter.Label(root,textvariable=text1, width=500)
label1.pack(side=tkinter.TOP)

label2=tkinter.Label(root,textvariable=text2, width=500)
label2.pack(side=tkinter.TOP)

label3=tkinter.Label(root,textvariable=text3, width=500)
label3.pack(side=tkinter.TOP)

label4=tkinter.Label(root,textvariable=text4, width=500)
label4.pack(side=tkinter.TOP)

label5=tkinter.Label(root,textvariable=text5, width=500)
label5.pack(side=tkinter.TOP)

label6=tkinter.Label(root,textvariable=text6, width=500)
label6.pack(side=tkinter.TOP)

#frames to orgranize buttons
frame1=tkinter.Frame(
    root
)
frame2=tkinter.Frame(
    root
)
frame3=tkinter.Frame(
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
    text="Hold",
    bg="grey",
    command=pubH
)

Zig=tkinter.Button(
    frame3,
    text="Go to Delivery",
    bg="grey",
    # command=pubZ
)

spring=tkinter.Button(
    frame1,
    text="Return from Delivery",
    bg="grey",
    # command=pubS
)

dance=tkinter.Button(
    frame1,
    text="Dance",
    bg="grey",
    command=pubD
)

none=tkinter.Button( #an empty button which would have a different command but I ran out of time
    frame3,
    text="Empty",
    bg="red"
    # command=pubF
)

#packing frames onto tkinter window
frame1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
frame2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
frame3.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)

#packing buttons onto the frames in a percise order so it looks right
spring.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
forward.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
Zig.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
left.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
hold.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
right.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
dance.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
back.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
none.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)






# rate = rospy.Rate(10)
root.mainloop() #tktiner mainloopa
# while  not rospy.is_shutdown(): #Replacement mainloop
#     root.update_idletasks()
#     root.update()
#     rate.sleep()
