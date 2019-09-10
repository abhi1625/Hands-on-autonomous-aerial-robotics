#! /usr/bin/env python3


from pyquaternion import Quaternion
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import math
import scipy.io
import sys
import pandas as pd
pd.set_option('display.float_format', lambda x: '%.4f' % x)
import csv




def quat_to_euler(q):
    w,x,y,z = q
    R11 = 2*(w**2)-1+2*(x**2)
    R21 = 2*(x*y-w*z)
    R31 = 2*(x*z+w*y)
    R32 = 2*(y*z-w*x)
    R33 = 2*(w**2)-1+2*(z**2)
    if R31**2 >=1.0:
    	# print "true"
    	R31 = 0.99499
    
    phi = math.atan2(R32, R33 )*(180/np.pi);
    theta = -math.atan(R31/math.sqrt(1-R31**2) )*(180/np.pi);
    psi = math.atan2(R21, R11 )*(180/np.pi);
    return phi, theta, psi


def quaternConj(q):
    w,x,y,z = q
    q_conj = np.array([w,-x,-y,-z])
    return q_conj



class madgwick:
	def __init__(self,q=Quaternion(1,0,0,0),beta = 0.9,invSampleFreq = 1.0/100.0 ):
		self.beta = beta
		w,x,y,z = q 
		self.q_new = np.array([[w],[x],[y],[z]])
		self.invSampleFreq = invSampleFreq
		self.counter = 0
		self.q_acc = np.array([[1],[0],[0],[0]],dtype = np.float32)
		self.q_gyro = np.array([[1],[0],[0],[0]],dtype = np.float32)
    
	def imu_update(self,acc,gyro,q_est):
		'''
		INPUT:
		    acc= numpy array of length 3 having three accelerations ax, ay, az
		    gyro= numpy array of length 3 having three angular accelerations gx, gy, gz

		Output:
		    Pose in quaternion
		'''

		if(not(np.linalg.norm(acc)==0)):
			#normalize acc data
			acc = np.divide(acc,np.linalg.norm(acc))
			ax = acc[0]
			ay = acc[1]
			az = acc[2]
			q1,q2,q3,q4 = q_est

			f = np.array([2*(q2*q4-q1*q3) - ax,
			              2*(q1*q2+q3*q4) - ay,
			              2*(0.5 - q2**2 - q3**2) - az])

			J = np.matrix([[-2*q3, 2*q4, -2*q1, 2*q2],
			               [2*q2, 2*q1, 2*q4, 2*q3],
			               [0, -4*q2, -4*q3, 0]])

			step = np.array(np.matmul(J.T,f))
			step = np.divide(step,np.linalg.norm(step))

			q_del = self.beta*step
			gyro_quat = Quaternion(0,gyro[0],gyro[1],gyro[2])
			q_dot_ob = (q_est*gyro_quat)
			q_dot = 0.5*q_dot_ob.q
			q_dot = np.reshape(q_dot,(4,1))
			# print("step = ", step, step.shape)
			self.q_acc += step*self.invSampleFreq
			self.q_gyro += q_dot*self.invSampleFreq


		q_dot_est_new = q_dot-q_del
		self.q_new += q_dot_est_new*self.invSampleFreq
		self.q_new = np.divide(self.q_new,np.linalg.norm(self.q_new))
		np.set_printoptions(precision=5)

		return np.reshape(self.q_new,(4,)),self.q_acc,self.q_new



def process_data(acc_data,acc_scale,acc_bias,gyro):

    acc_data = np.multiply(acc_data,acc_scale[:,np.newaxis])
    acc_data = acc_data + acc_bias[:,np.newaxis]

    gyro_bias = np.mean(gyro[:,:200],axis = 1)

    gyro_data = gyro - gyro_bias[:,np.newaxis]
    gyro_data = (3300/1023)*(np.pi/180)*(1/3.33)*gyro_data[[2,0,1],:]

    return acc_data, gyro_data

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))
                

def main():
	imu_params = scipy.io.loadmat('./IMUParams.mat')
	acc_scale = imu_params['IMUParams'][0]
	acc_bias = imu_params['IMUParams'][1]
	data = scipy.io.loadmat('./YourDirectoryID_p1a/Data/Train/IMU/imuRaw4.mat')
	imu_data_vicon = scipy.io.loadmat('./YourDirectoryID_p1a/Data/Train/Vicon/viconRot4.mat')

	acc = data['vals'][:3,:]
	acc = np.float32(acc)
	gyro = np.float32(data['vals'][-3:,:])

	# print gyro.shape
	# input('a')
	acc_all,gyro_all = process_data(acc,acc_scale,acc_bias,gyro)
	q_est = Quaternion(1,0,0,0)
	q_init = Quaternion(1,0,0,0)
	angles = np.array([0,0,0])
	acc_pose = np.array([0,0,0])
	gyro_pose = np.array([0,0,0])
	mk=madgwick(q = q_init)
	thresh = acc_all.shape[1] -1
	i=0

	while i<=thresh:
		acc = np.reshape(acc_all[:,i], (3,1))
		gyro = np.reshape(gyro_all[:,i], (3,1))

		q, acc_int, gyro_int = mk.imu_update(acc,gyro,q_est)

		phi,thetha,psy = quat_to_euler(quaternConj(q))
		temp = np.array([phi, thetha, psy])
		angles= np.vstack((angles,temp))

		phi,thetha,psy = quat_to_euler(quaternConj(acc_int))
		temp = np.array([phi, thetha, psy])
		acc_pose = np.vstack((acc_pose, temp))

		phi,thetha,psy = quat_to_euler(quaternConj(gyro_int))
		temp = np.array([phi, thetha, psy])
		gyro_pose = np.vstack((gyro_pose, temp))

		q_est = Quaternion(q[0],q[1],q[2],q[3])
		i+=1

	angles = angles[1:]
	num=0
	acc_all = acc_all[num:]
	gyro_all = gyro_all[num:]
	#PLot vicon data
	rots = imu_data_vicon['rots']
	vicon_data = np.zeros((3,rots.shape[2]),dtype=np.float32)
	for ang in range(rots.shape[2]):
	    vicon_data[:,ang] = (180/np.pi)*rot2eul(rots[:,:,ang])

	# print(vicon_data.shape)
	t_vicon = np.linspace(1,vicon_data.shape[1],num=vicon_data.shape[1])
	# input('aaaa')
	    
	t = np.linspace(1,int(acc_all.shape[1]),num = int(acc_all.shape[1]))
	t_angles = np.linspace(1,int(angles.shape[0]),num = int(angles.shape[0]))
	fig=plt.figure(1)
	# plt.rcParams['figure.figsize'] = [10,10]

	a1 = plt.subplot(3,4,1)
	plt.plot(t[:thresh],acc_all[0,:thresh],'r-')
	a1.title.set_text('X-accel')

	a2 = plt.subplot(3,4,2)
	plt.plot(t[:thresh],gyro_all[0,:thresh],'r-')
	a2.title.set_text('X-gyro')

	# print t.shape,angles[:,0].shape

	a3 = plt.subplot(3,4,3)
	plt.plot(t_angles,angles[:,0],'r-')
	a3.title.set_text('X-madgwick')

	a10 = plt.subplot(3,4,4)
	plt.plot(t_vicon,vicon_data[0,:],'r-')
	a10.title.set_text('X-madgwick')

	a4 = plt.subplot(3,4,5)
	plt.plot(t[:thresh],acc_all[1,:thresh],'g-')
	a4.title.set_text('Y-accel')

	a5 = plt.subplot(3,4,6)
	plt.plot(t[:thresh],gyro_all[1,:thresh],'g-')
	a5.title.set_text('Y-gyro')

	a6 = plt.subplot(3,4,7)
	plt.plot(t_angles,angles[:,1],'g-')
	a6.title.set_text('Y-madgwick')

	a11 = plt.subplot(3,4,8)
	plt.plot(t_vicon,vicon_data[1,:],'g-')
	a11.title.set_text('Y-madgwick')

	a7 = plt.subplot(3,4,9)
	plt.plot(t[:thresh],acc_all[2,:thresh],'b-')
	a7.title.set_text('Z-accel')

	a8 = plt.subplot(3,4,10)
	plt.plot(t[:thresh],gyro_all[2,:thresh],'b-')
	a8.title.set_text('Z-gyro')

	a9 = plt.subplot(3,4,11)
	plt.plot(t_angles,angles[:,2],'b-')
	a9.title.set_text('Z-madgwick')

	a12 = plt.subplot(3,4,12)
	plt.plot(t_vicon,vicon_data[2,:],'b-')
	a12.title.set_text('Z-madgwick')

	# print(len(angles))


	plt.show()


if __name__ == '__main__':
	main()