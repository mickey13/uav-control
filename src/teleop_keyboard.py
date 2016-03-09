#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

helpMessage = """
Use keyboard to move UAV.
---------------------------
k: hover
a: ascend
d: descend
l: + x-axis
j: - x-axis
i: + y-axis
,: - y-axis
w: warp

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

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

rospy.init_node('teleop_keyboard')
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
settings = termios.tcgetattr(sys.stdin)

print helpMessage

while(1):
  key = getKey()

  if key in moveBindings.keys():
    twist = Twist()
    twist.linear.x = moveBindings[key][0]
    twist.linear.y = moveBindings[key][1]
    twist.linear.z = moveBindings[key][2]
    pub.publish(twist)
    print twist
  elif (key == '\x03'):
    break

  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

