#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance:
def __init__(self):
self.active = False
rospy.Subscriber('/scan', LaserScan, self.scan_callback)
self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def scan_callback(self, msg):
if np.min(msg.ranges) < 0.6:
self.active = True
cmd = Twist()
cmd.linear.x = 0.0
cmd.angular.z = 0.8
self.cmd_pub.publish(cmd)
else:
self.active = False