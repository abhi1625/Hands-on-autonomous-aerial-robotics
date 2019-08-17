#!/usr/bin/env python
# Unscented Kalman Filter Code for attitude estimation
# Author(s): Prateek Arora(pratique@terpmail.umd.edu)

import numpy as np 
import math


class UKF:
	def __init__(self):
	# state vector x = [q,w]
	self.x = np.array([1,0,0,0,0,0,0])
	# Process Model noise
	self.Q = np.zeros((6,6))
	np.fill_diagonal(self.Q, [10,10,10,0.1,0.1,0.1])
	# Measurement Model noise
	self.R = np.zeros((6,6))
	np.fill_diagonal(self.R, [1,1,1,0.001,0.001,0.001])
	# Convariance Matrix
	self.P_k= np.zeros((6,6))
	np.fill_diagonal(self.R, [1, 1, 1, 1, 1, 1])
	#number of independent states
	self.n = 6

	def Update():
		#  sigma points
		self.S = np.linalg.cholesky(self.P_k+Q)
