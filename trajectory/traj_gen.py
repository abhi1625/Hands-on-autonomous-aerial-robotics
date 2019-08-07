#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist


def circle_traj():
	t=0
	dt = 0.01 
	Kp = [1.8,1,1.5]	# [x,y,z]
	radius = 1 # 0.5m
	T = 1 #time taken to complete a circle
	pi = np.pi
	
	NumTurns = 1

	angle = 2*pi*t/T

	# initialize node
	rospy.init_node('Traj', anonymous=True)
	#Publisher
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
	
	velocity_msg = Twist()
	r = rospy.Rate(10)

	x_prev = 0
	y_prev = -radius
	z_prev = 0

	x=0
	y=0
	z=0
	count =1 
	
	while ((not rospy.is_shutdown()) and (angle<=NumTurns*2*pi)):
		angle = 2*pi*t/T

		x = radius*math.cos(angle-pi/2)
		y = radius*math.sin(angle-pi/2)

		del_x = x - x_prev
		del_y = y - y_prev
		del_z = z - z_prev


		velocity_msg.linear.x = Kp[0]*del_x
		velocity_msg.linear.y = Kp[1]*del_y
		velocity_msg.linear.z = Kp[2]*del_z

		print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

		#Update terms
		x_prev = x
		y_prev = y
		z_prev = z
		t+=dt
		r.sleep()

	print('end')





def rhombus_traj():
		
	t=0
	dt = 0.1
	
	Kp = [1.8, 1, 2.0] # [x,y,z]
	radius = 1 # 0.5m
	T = 1 #time taken to complete a circle
	pi = np.pi

	NumTurns = 1

	angle = 2*pi*t/T
	#Publisher
	rospy.init_node('Traj', anonymous=True)
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
	velocity_msg = Twist()
	#z = 0.05
	r = rospy.Rate(10)
	#n=0
	x_prev = 0
	y_prev = -radius
	z_prev = 0
	count =1 
	x=0
	y=0
	z=0
	while ((not rospy.is_shutdown()) and (t<=2)):          
		angle = 2*pi*t/T
		
		if count ==1:
			x = t 
			y = t
			z = t
			del_x = x - x_prev
			del_y = y - y_prev
			del_z = z - z_prev
			if t>=1.9:
				count+=1
				t=0
				x_prev=0
				y_prev=0
				z_prev=0
		if count ==2:
			x = -t 
			y = t
			z = t
			del_x = x - x_prev
			del_y = y - y_prev
			del_z = z - z_prev
			if t>=1.9:
				count+=1
				t=0
				x_prev=0
				y_prev=0
				z_prev=0
		if count ==3:
			x = -t 
			y = -t
			z = -t
			del_x = x - x_prev
			del_y = y - y_prev
			del_z = z - z_prev
			if t>=1.9:
				count+=1
				t=0
				x_prev=0
				y_prev=0
				z_prev=0
		if count ==4:
			x = t 
			y = -t
			z = -t
			del_x = x - x_prev
			del_y = y - y_prev
			del_z = z - z_prev
			#if t>=0.9:
			#	count+=1

		#Update terms
		x_prev = x
		y_prev = y
		z_prev = z

		velocity_msg.linear.x = Kp[0]*del_x
		velocity_msg.linear.y = Kp[1]*del_y
		velocity_msg.linear.z = Kp[2]*del_z

		print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

		pub.publish(velocity_msg)
		
		t+=dt
		r.sleep()

	print('end')




def sawtooth_traj():
	t=0
	dt = 0.1
	Kp = [1.8,1,1.5]	
	radius = 1 # 0.5m
	T = 1 #time taken to complete a circle
	pi = np.pi
	NumTurns = 1
	angle = 2*pi*t/T

	#Publisher
	rospy.init_node('Traj', anonymous=True)
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
	velocity_msg = Twist()
	#z = 0.05
	r = rospy.Rate(10)
	#n=0
	x_prev = 0
	y_prev = -radius
	z_prev = 0
	count =1 
	x=0
	y=0
	z=0
	while ((not rospy.is_shutdown()) and (count<5)):
		angle = 2*pi*t/T

		if count%2 ==0:
			del_x = -1.2*dt
			del_y = dt
			del_z = dt
			if t>1.9:
				count+=1
				t=0
				del_x = 0
				del_y = 0
				del_z = 0
		else:
			del_x = 1.2*dt
			del_y = dt
			del_z = dt
			if t>1.9:
				count+=1
				t=0
				del_x = 0
				del_y = 0
				del_z = 0
		
	

		velocity_msg.linear.x = Kp[0]*del_x
		velocity_msg.linear.y = Kp[1]*del_y
		velocity_msg.linear.z = Kp[2]*del_z

		print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

		pub.publish(velocity_msg)
		
		#Update terms
		x_prev = x
		y_prev = y
		z_prev = z
		t+=dt
		r.sleep()

	print('end')



if __name__ == '__main__':
	circle_traj()
	# rhombus_traj()
	# sawtooth_traj()