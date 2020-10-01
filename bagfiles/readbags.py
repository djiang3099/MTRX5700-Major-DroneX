#!/usr/bin/env python

import rosbag
import rospy
import math
from tf.transformations import euler_from_quaternion
import csv
import matplotlib.pyplot as plt

bag = rosbag.Bag("bagfiles/sun_N_007_005_pYaw.bag")
print("Starting...")

#variable declaration
xVect = []
yVect = []
zVect = []
yawVect = []
t_odom_Vect = []
com_x_vect = []
com_y_vect = []
com_z_vect = []
com_az_vect = []
t_com_Vect = []
goal_x_vect = []
goal_y_vect = []
goal_z_vect = []
goal_az_vect = []
t_goal_vect = []

first = True
t_first = 0

for topic, msg, t in bag.read_messages(topics=["/dronex/odom"]):
	quat = msg.pose.pose.orientation
	roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z

	if first:
		t_first = t.to_time()
		first = False

	xVect.append(x)
	yVect.append(y)
	zVect.append(z)
	yawVect.append(yaw)

	t_odom_Vect.append(t.to_time() - t_first)


first = True
for topic, msg, t in bag.read_messages(topics=["/cmd_vel"]):
	com_x = msg.linear.x
	com_y = msg.linear.y
	com_z = msg.linear.z
	com_az = msg.angular.z

	com_x_vect.append(com_x)
	com_y_vect.append(com_y)
	com_z_vect.append(com_z)
	com_az_vect.append(com_az)

	t_com_Vect.append(t.to_time() - t_first)

first = True
for topic, msg, t in bag.read_messages(topics=["/droneGoal"]):
	goal_x = msg.position.x
	goal_y = msg.position.y
	goal_z = msg.position.z

	quat = msg.orientation
	roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	goal_az = yaw

	goal_x_vect.append(goal_x)
	goal_y_vect.append(goal_y)
	goal_z_vect.append(goal_z)
	goal_az_vect.append(goal_az)

	t_goal_vect.append(t.to_time() - t_first)

plt.plot(t_odom_Vect, xVect , label = 'odom X')
plt.hold(True)
plt.plot(t_odom_Vect, yVect , label = 'odom Y')
plt.plot(t_odom_Vect, zVect , label = 'odom Z')
plt.plot(t_odom_Vect, yawVect , label = 'odom Yaw')

plt.plot(t_com_Vect, com_x_vect , label = 'com_vel x')
plt.plot(t_com_Vect, com_y_vect , label = 'com_vel y')
plt.plot(t_com_Vect, com_z_vect , label = 'com_vel z')
plt.plot(t_com_Vect, com_az_vect , label = 'com_vel az')

plt.hlines(goal_x, t_odom_Vect[0], t_odom_Vect[-1], label = 'x goal')
plt.hlines(goal_y, t_odom_Vect[0], t_odom_Vect[-1], label = 'y goal')
plt.hlines(goal_z, t_odom_Vect[0], t_odom_Vect[-1], label = 'z goal')
plt.hlines(goal_az, t_odom_Vect[0], t_odom_Vect[-1], label = 'az goal')
legend = plt.legend()

plt.show()
#cleanup
bag.close()
print("End")
