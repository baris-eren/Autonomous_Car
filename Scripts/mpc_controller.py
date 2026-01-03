#!/usr/bin/env python3
import rospy
import numpy as np
import cvxpy as cp
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class RealMPCController:
def __init__(self):
self.dt = 0.1
self.N = 10 # horizon
self.state = np.zeros(3) # x, y, theta
self.path = []
self.index = 0


rospy.Subscriber('/odom', Odometry, self.odom_callback)
rospy.Subscriber('/global_path', Path, self.path_callback)
self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def odom_callback(self, msg):
self.state[0] = msg.pose.pose.position.x
self.state[1] = msg.pose.pose.position.y
q = msg.pose.pose.orientation
_, _, self.state[2] = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


def path_callback(self, msg):
self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
self.index = 0


def solve_mpc(self, ref):
x = cp.Variable((3, self.N+1))
u = cp.Variable((2, self.N)) # v, w


cost = 0
constraints = [x[:,0] == self.state]


for k in range(self.N):
cost += cp.sum_squares(x[0,k] - ref[0])
cost += cp.sum_squares(x[1,k] - ref[1])
cost += 0.1 * cp.sum_squares(u[:,k])


x_next = cp.hstack([
x[0,k] + u[0,k]*cp.cos(x[2,k])*self.dt,
x[1,k] + u[0,k]*cp.sin(x[2,k])*self.dt,
x[2,k] + u[1,k]*self.dt
])


constraints += [x[:,k+1] == x_next]
constraints += [u[0,k] <= 0.5, u[0,k] >= 0.0]
constraints += [cp.abs(u[1,k]) <= 1.0]


prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve(solver=cp.OSQP)


return u.value[:,0]


def control_step(self):
if not self.path or self.index >= len(self.path):
return


ref = self.path[self.index]
u = self.solve_mpc(ref)


cmd = Twist()
cmd.linear.x = float(u[0])
cmd.angular.z = float(u[1])
self.cmd_pub.publish(cmd)


dist = np.hypot(ref[0] - self.state[0], ref[1] - self.state[1])
if dist < 0.2:
self.index += 1