#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

x_prev = 0
y_prev = 0
t_prev = 0

def circle_traj():
	t = 0.001 # 0.05sec
	radius = 0.5 # 0.5m
	T = 1 #time taken to complete a circle
	pi = np.pi

	global x_prev,y_prev,t_prev

	#list for plotting
	x_plot = []
	y_plot = []
	t_plot = []
	
	del_x_plot = []
	del_y_plot = []

	angle = 2*pi*t/T
	while angle<2*pi:
		angle = 2*pi*t/T
		# print("angle = ",angle)
		# print("cos = ",math.cos(angle))
		x = radius*math.cos(angle)
		y = radius*math.sin(angle)
		
		del_x = x - x_prev
		del_y = y - y_prev
		del_z=0
		
		x_plot.append(x)
		y_plot.append(y)
		del_x_plot.append(del_x)
		del_y_plot.append(del_y)
		t_plot.append(t)

		#Update terms
		x_prev = x
		y_prev = y
		t+=0.01

	# print("before plotting cmd")
	plt.plot(del_y_plot,del_x_plot, 'g*')
	plt.plot(y_plot,x_plot, 'c*')
	plt.plot(t_plot,x_plot, 'ro')
	plt.plot(t_plot,y_plot, 'bo')
	# plt.plot(t_plot,del_y_plot, 'yo')
	plt.xlabel("Time (s)")
	plt.ylabel("traj")
	# print("before show")
	plt.show()

if __name__ == '__main__':
	circle_traj()

