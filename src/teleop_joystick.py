#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

helpMessage = """
Use joystick to move UAV.
---------------------------
'Start': take off (AR Drone)
'Select': land (AR Drone)
'L1': rotate counterclockwise
'R1': rotate clockwise
'Left Stick Up': ascend
'Left Stick Down': descend
'Right Stick Up': move forward
'Right Stick Down': move backward
'Right Stick Left': move left
'Right Stick Right': move right

CTRL-C to quit
"""

def joystickCallback(msg):
  twist = Twist()
  if (msg.buttons[4]):
    twist.angular.z = 1.0;
  elif (msg.buttons[5]):
    twist.angular.z = -1.0;
  twist.linear.x = msg.axes[3]
  twist.linear.y = msg.axes[2]
  twist.linear.z = msg.axes[5]
  pubMove.publish(twist)
  if (msg.buttons[7]):
    pubEmergency.publish(Empty())
  elif (msg.buttons[8]):
    pubStop.publish(Empty())
  elif (msg.buttons[9]):
    pubStart.publish(Empty())
  return

rospy.init_node('teleop_joystick')
sub = rospy.Subscriber('/joy', Joy, joystickCallback)
pubMove = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
pubStart = rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 10)
pubStop = rospy.Publisher('/ardrone/land', Empty, queue_size = 10)
pubEmergency = rospy.Publisher('/ardrone/reset', Empty, queue_size = 10)

print helpMessage

rospy.spin()
