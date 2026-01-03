#!/usr/bin/env python3
import rospy
from a_star_planner import AStarPlanner
from mpc_controller import RealMPCController
from obstacle_avoidance import ObstacleAvoidance


if __name__ == '__main__':
rospy.init_node('turtlebot_autonomy_real_mpc')
planner = AStarPlanner()
controller = RealMPCController()
obstacle = ObstacleAvoidance()


rate = rospy.Rate(10)
rospy.sleep(2)


start = (10, 10)
goal = (80, 80)


while planner.map is None and not rospy.is_shutdown():
rospy.sleep(0.5)


came_from = planner.plan(start, goal)
planner.publish_path(came_from, start, goal)


while not rospy.is_shutdown():
if not obstacle.active:
controller.control_step()
rate.sleep()