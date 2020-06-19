#!/usr/bin/env python

import rosbag
import rospy
import math
from tf.transformations import euler_from_quaternion
import csv
import matplotlib.pyplot as plt

params = {'xtick.labelsize': 'x-large',
	  'ytick.labelsize': 'x-large',
	'axes.labelsize': 'x-large'}
plt.rcParams.update(params)

#bag = rosbag.Bag("path1.bag")
#bag = rosbag.Bag("path2.bag")
#bag = rosbag.Bag("SLAMBag.bag")
bag = rosbag.Bag("withVideo/final?/wed_c_gesture.bag")
print("Starting...")
# rospy.init_node("readBags")

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

drone_odom_x = []
drone_odom_y = []
drone_odom_z = []
drone_odom_yaw = []
t_drone_odom = []


first = True
t_first = 0
for topic, msg, t in bag.read_messages(topics=["/dronex/odom"]):
	quat = msg.pose.pose.orientation
	roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	#print ("{},{}".format(  rospy.get_rostime()  ,yaw))
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z

	if first:
		t_first = t.to_time()
		# first = False

	xVect.append(x)
	yVect.append(y)
	zVect.append(z)
	yawVect.append(yaw*180/3.1415)

	t_odom_Vect.append(t.to_time() - t_first)

	#rospy.sleep(0.01)

for topic, msg, t in bag.read_messages(topics=["/ardrone/odometry"]):
	quat = msg.pose.pose.orientation
	roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	#print ("{},{}".format(  rospy.get_rostime()  ,yaw))
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z

	if first:
		t_first = t.to_time()
		t_first += 20
		first = False

	drone_odom_x.append(x)
	drone_odom_y.append(y)
	drone_odom_z.append(z)
	drone_odom_yaw.append(yaw*180/3.1415)

	t_drone_odom.append(t.to_time() - t_first)

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

#for topic, msg, t in bag.read_messages(topics=["/scan"]):


# plt.plot(t_odom_Vect, xVect , label = 'odom X')
# plt.hold(True)
# plt.plot(t_odom_Vect, yVect , label = 'odom Y')
# plt.plot(t_odom_Vect, zVect , label = 'odom Z')
# # plt.plot(t_odom_Vect, yawVect , label = 'odom Yaw')

# plt.plot(t_com_Vect, com_x_vect , label = 'com_vel x')
# plt.plot(t_com_Vect, com_y_vect , label = 'com_vel y')
# plt.plot(t_com_Vect, com_z_vect , label = 'com_vel z')
# #plt.plot(t_com_Vect, com_az_vect , label = 'com_vel az')

# plt.hlines(goal_x, t_odom_Vect[0], t_odom_Vect[-1], label = 'x goal')
# plt.hlines(goal_y, t_odom_Vect[0], t_odom_Vect[-1], label = 'y goal')
# plt.hlines(goal_z, t_odom_Vect[0], t_odom_Vect[-1], label = 'z goal')
# #plt.hlines(goal_az, t_odom_Vect[0], t_odom_Vect[-1], label = 'az goal')

plt.plot(t_drone_odom, drone_odom_x , label = 'Odom X')
plt.hold(True)
plt.plot(t_drone_odom, drone_odom_y , label = 'Odom Y')
plt.plot(t_drone_odom, drone_odom_z , label = 'Odom Z')

legend = plt.legend()

plt.xlabel('Time(s)', fontsize=18)
plt.ylabel('Magnitude (m)', fontsize=18)

plt.show()
#rospy.sleep(5.)

bag.close()
print("End")
