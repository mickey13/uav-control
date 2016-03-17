#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import sys, select, termios, tty

helpMessage = """
Use keyboard to move UAV.
---------------------------
'1': take off (AR Drone)
'0': land (AR Drone)
'w': rotate counterclockwise
'r': rotate clockwise
'e': stop rotating
'a': ascend
'd': descend
'i': move forward
',': move backward
'j': move left
'l': move right
'k': hover / stop moving

CTRL-C to quit
"""

moveBindings = {
  'k' : (0.0, 0.0, 0.0),
  'a' : (0.0, 0.0, 1.0),
  'd' : (0.0, 0.0, -1.0),
  'l' : (0.0, -1.0, 0.0),
  'j' : (0.0, 1.0, 0.0),
  'i' : (1.0, 0.0, 0.0),
  ',' : (-1.0, 0.0, 0.0)
}

rotateBindings = {
  'e' : (0.0, 0.0, 0.0),
  'w' : (0.0, 0.0, 1.0),
  'r' : (0.0, 0.0, -1.0)
}

takeOffKey = '1'
landKey = '0'

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

rospy.init_node('teleop_keyboard')
pubMove = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
pubStart = rospy.Publisher('/ardrone/takeoff', Empty, queue_size = 10)
pubStop = rospy.Publisher('/ardrone/land', Empty, queue_size = 10)
settings = termios.tcgetattr(sys.stdin)

print helpMessage

while(1):
  key = getKey()

  if (key in moveBindings.keys()):
    twist = Twist()
    twist.linear.x = moveBindings[key][0]
    twist.linear.y = moveBindings[key][1]
    twist.linear.z = moveBindings[key][2]
    pubMove.publish(twist)
  if (key in rotateBindings.keys()):
    twist = Twist()
    twist.angular.z = rotateBindings[key][2]
    pubMove.publish(twist)
  if (key == takeOffKey):
    pubStart.publish(Empty())
  if (key == landKey):
    pubStop.publish(Empty())
  if (key == '\x03'):
    break

  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

