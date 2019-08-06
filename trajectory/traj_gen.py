#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

#x_prev = 0
#y_prev = 0
#t_prev = 0

def circle_traj():
	circle_flag=False
	rhombus_flag=False
	sawtooth_flag=True
	t=0
	#dt = 0.01 circle
	#dt = 0.1  rhombus
	dt = 0.1 #sawtooth  
	#Kp = [1.8, 1, 2.0] # [x,y,z]
	Kp = [1.8,1,1.5]	
	radius = 1 # 0.5m
	T = 1 #time taken to complete a circle
	pi = np.pi

	#global x_prev,y_prev,t_prev

	#list for plotting
	x_plot = []
	y_plot = []
	t_plot = []
	
	del_x_plot = []
	del_y_plot = []
	
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
	while ((not rospy.is_shutdown()) and (count<5)):    #(t<=2)):          #(angle<=NumTurns*2*pi)):
		#while ((not rospy.is_shutdown()) and (n<10)):
		angle = 2*pi*t/T
		# print("angle = ",angle)
		# print("cos = ",math.cos(angle))
		if circle_flag:
			x = radius*math.cos(angle-pi/2)
			y = radius*math.sin(angle-pi/2)

			del_x = x - x_prev
			del_y = y - y_prev
			del_z = z - z_prev
		
		elif rhombus_flag:
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

 		elif sawtooth_flag:
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
			
		
		#x_plot.append(x)
		#y_plot.append(y)
		del_x_plot.append(del_x)
		del_y_plot.append(del_y)
		t_plot.append(t)

		#Update terms
		x_prev = x
		y_prev = y
		z_prev = z

		velocity_msg.linear.x = Kp[0]*del_x
		velocity_msg.linear.y = Kp[1]*del_y
		velocity_msg.linear.z = Kp[2]*del_z

		#velocity_msg.linear.x = z
		#velocity_msg.linear.y = 0
		#velocity_msg.linear.z = 0

		#print(x,y)
		print(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z)

		pub.publish(velocity_msg)
		
		t+=dt
		print(t)
		r.sleep()
	# print("before plotting cmd")
	#plt.plot(del_y_plot,del_x_plot, 'g*')
	#plt.plot(y_plot,x_plot, 'c*')
	#plt.plot(t_plot,x_plot, 'ro')
	#plt.plot(t_plot,y_plot, 'bo')
	# plt.plot(t_plot,del_y_plot, 'yo')
	#plt.xlabel("Time (s)")
	#plt.ylabel("traj")
	# print("before show")
	#plt.show()
	print('end')

if __name__ == '__main__':
	circle_traj()
