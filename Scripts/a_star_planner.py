#!/usr/bin/env python3
import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class AStarPlanner:
def __init__(self):
self.map = None
self.resolution = None
self.width = None
self.height = None
self.origin = None
rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1)


def map_callback(self, msg):
self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
self.resolution = msg.info.resolution
self.width = msg.info.width
self.height = msg.info.height
self.origin = msg.info.origin


def heuristic(self, a, b):
return abs(a[0]-b[0]) + abs(a[1]-b[1])


def neighbors(self, node):
x, y = node
for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
nx, ny = x+dx, y+dy
if 0 <= nx < self.width and 0 <= ny < self.height:
if self.map[ny][nx] == 0:
yield (nx, ny)


def plan(self, start, goal):
open_set = []
heapq.heappush(open_set, (0, start))
came_from = {}
cost = {start: 0}


while open_set:
_, current = heapq.heappop(open_set)
if current == goal:
break
for n in self.neighbors(current):
new_cost = cost[current] + 1
if n not in cost or new_cost < cost[n]:
cost[n] = new_cost
priority = new_cost + self.heuristic(goal, n)
heapq.heappush(open_set, (priority, n))
came_from[n] = current
return came_from


def publish_path(self, came_from, start, goal):
path = Path()
path.header.frame_id = 'map'
current = goal
while current != start:
pose = PoseStamped()
pose.pose.position.x = current[0] * self.resolution
pose.pose.position.y = current[1] * self.resolution
path.poses.append(pose)
current = came_from[current]
self.path_pub.publish(path)