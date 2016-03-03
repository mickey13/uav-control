#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

helpMessage = """
Use joystick to move UAV.
---------------------------
'L': ascend
'R': descend
'Right Stick Up': move forward
'Right Stick Down': move backward
'Right Stick Left': move left
'Right Stick Right': move right

CTRL-C to quit
"""

def joystickCallback(msg):
  twist = Twist()
  if (msg.buttons[4]):
    twist.linear.z = 1.0
  elif (msg.buttons[5]):
    twist.linear.z = -1.0
  twist.linear.x = msg.axes[2]
  twist.linear.y = -1.0 * msg.axes[3]
  pub.publish(twist)
  return 

rospy.init_node('teleop_joystick')
sub = rospy.Subscriber('joy', Joy, joystickCallback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

print helpMessage

rospy.spin()

