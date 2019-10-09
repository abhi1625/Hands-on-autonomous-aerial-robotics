#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
# step size(dt) = 0.5 @ 10 hz publish rate = 1 meter dist travel
class pid_controller :
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.inFlight = False
		self.pose = Pose()
		self.twist = Twist()
		self.bias_x = 0
		self.bias_y = 0
		self.bias_z = 0
		self.prev_error = 0

	def calculate_bias(self):
		self.bias_x = self.pose.position.x
		self.bias_y = self.pose.position.y
		self.bias_z = self.pose.position.z	

	def takeoff(self):
		self.takeoff_pub.publish()
		self.inFlight = True

	def odom_callback(self, data):
		self.pose = data.pose.pose
		# print "pose ", self.pose
		self.Twist = data.twist.twist
		# print "twist ", self.Twist

	def straight_line(self):
		
		t=0
		dt = 0.1
		
		Kp = [0.15, 1.0, 1.0] # [x,y,z]
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
		transition_rate = rospy.Rate(5)
		#n=0
		x_prev = 0
		y_prev = -radius
		z_prev = 0
		count =1 
		x=0
		y=0
		z=0
		x = 0
		time_begin = rospy.Time.now()


		while ((not rospy.is_shutdown()) and x<=100):          
			# angle = 2*pi*t/T
			if self.inFlight != True:
				print("in flight")
				self.takeoff()
				time.sleep(3)
				input('a')

				self.calculate_bias()
				bias_x = self.bias_x
				bias_y = self.bias_y
				bias_z = self.bias_z
				print("x bias = ",self.bias_x)
				print("y bias = ",self.bias_y)
				print("z bias = ",self.bias_z)
				input('aaaaa')
			
			if(x<=50):
				x_des = -1
				y_des = 0
				z_des = 0
				x_odom = self.pose.position.x - bias_x
				y_odom = self.pose.position.y - bias_y
				z_odom = self.pose.position.z - bias_z
				
				# del_x = 0
				# del_y = dt
				# del_z = 0


				
				err_x = x_des - x_odom

				#Update terms
				velocity_msg.linear.x = - Kp[0]*(err_x) + 0.0001*(err_x - self.prev_error)/0.1
				velocity_msg.linear.y = Kp[1]*0
				velocity_msg.linear.z = Kp[2]*0
				print("x_odom = ", x_odom, "x_des = ", x_des)
				print("x_vel",self.twist.linear.x)
				print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

				pub.publish(velocity_msg)
				x += dt
				t+=dt
				self.prev_error = err_x
				r.sleep()
			elif(x>50 and x<=60):
				x += dt
				transition_rate.sleep()
			elif(x>200 and x<=300):
				del_x = 0
				del_y = -dt
				del_z = 0
				

				#Update terms
				velocity_msg.linear.x = Kp[0]*del_x
				velocity_msg.linear.y = Kp[1]*del_y
				velocity_msg.linear.z = Kp[2]*del_z

				print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

				pub.publish(velocity_msg)
				x += dt
				t+=dt
				r.sleep()

			elif(x>300 and x<=400):
				x += dt
				transition_rate.sleep()

			elif(x>400 and x<=500):
				del_x = 0
				del_y = dt
				del_z = 0
				

				#Update terms
				velocity_msg.linear.x = Kp[0]*del_x
				velocity_msg.linear.y = Kp[1]*del_y
				velocity_msg.linear.z = Kp[2]*del_z

				print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

				pub.publish(velocity_msg)
				x += dt
				t+=dt
				# rospy.spinOnce()
				r.sleep()



		time_end = rospy.Time.now()
		diff = time_end.secs - time_begin.secs 
		print("total time  = ",diff )
		print('end')

def sin_xy():
		
	t=0
	dt = 0.1
	
	Kp = [4.5, 4.5, 1.0] # [x,y,z]
	radius = 1 # 0.5m
	T = 1 #time taken to complete a circle
	pi = np.pi

	NumTurns = 1

	angle = 2*pi*t/T
	#Publisher
	rospy.init_node('Traj', anonymous=True)
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
	velocity_msg = Twist()
	#z = 0.05
	r = rospy.Rate(10)
	transition_rate = rospy.Rate(5)
	#n=0
	x_prev = 0
	y_prev = -radius
	z_prev = 0
	count =1 
	x=0
	y=0
	z=0
	x = 0
	time_begin = rospy.Time.now()
	plot_y = []
	xt = []
	prev_y = 0
	while ((not rospy.is_shutdown()) and x<2.1):          
		y = 0.5*math.sin(math.pi*x)

		del_x = dt
		del_y = y - prev_y
		del_z = 0
		

		#Update terms
		velocity_msg.linear.x = Kp[0]*del_x
		velocity_msg.linear.y = Kp[1]*del_y
		velocity_msg.linear.z = Kp[2]*del_z

		print("del x = %.4f, del y = %.4f, del z = %.4f"%(velocity_msg.linear.x,velocity_msg.linear.y,velocity_msg.linear.z))

		pub.publish(velocity_msg)
		x += dt
		t+=dt
		prev_y = y

		plot_y.append(y)
		xt.append(x)
		r.sleep()


	# plt.plot(xt,plot_y)
	# plt.show()

	time_end = rospy.Time.now()
	diff = time_end.secs - time_begin.secs 
	print("total time  = ",diff )
	print('end')


if __name__ == '__main__':
	pid = pid_controller()
	pid.straight_line()
	# sin_xy()
	# rhombus_traj()
	# sawtooth_traj()