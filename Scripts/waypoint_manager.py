#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math


class WaypointManager:
def __init__(self):
rospy.init_node('waypoint_manager')
self.path_pub = rospy.Publisher('/mission_path', Path, queue_size=1)
self.file_path = rospy.get_param('~mission_file', 'mission.txt')
self.waypoints = self.load_mission()
self.publish_path()


def load_mission(self):
waypoints = []
with open(self.file_path, 'r') as f:
for line in f:
line = line.strip()
if not line or line.startswith('#'):
continue
# format: x y priority
parts = line.split()
x = float(parts[0])
y = float(parts[1])
priority = int(parts[2]) if len(parts) > 2 else 0
waypoints.append((priority, x, y))
# önceliğe göre sırala
waypoints.sort(key=lambda w: w[0])
return waypoints


def publish_path(self):
path = Path()
path.header.frame_id = 'map'
for _, x, y in self.waypoints:
pose = PoseStamped()
pose.pose.position.x = x
pose.pose.position.y = y
pose.pose.orientation.w = 1.0
path.poses.append(pose)
self.path_pub.publish(path)
rospy.loginfo('TXT mission yuklendi ve path publish edildi')


if __name__ == '__main__':
WaypointManager()
rospy.spin()