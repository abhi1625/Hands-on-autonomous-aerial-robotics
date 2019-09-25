from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation
import numpy as np
import math
from scipy.io import loadmat
import matplotlib.pyplot as plt
import tqdm
import sys
sys.path.append('/home/abhinav/Gits/drone-course/madgwick_filter/Code')
from madgwick import madgwick
# import pyquaternion as quat

np.set_printoptions(threshold = sys.maxsize)

class UKF:
    def __init__(self,init_state=np.array([1,0,0,0,0,0,0],dtype = np.float64)):

        self.dt = 1/100.0
        self.state = init_state                               #initial state
        #initial covariance matrix
        self.P = np.zeros((6,6))
        np.fill_diagonal(self.P, [0.01,0.01,0.01,0.01,0.01,0.01])
        # Process Model noise
        self.Q = np.zeros((6,6))
        np.fill_diagonal(self.Q, [100,100,100,0.05,0.05,0.05])
        # Measurement Model noise
        self.R = np.zeros((6,6))
        np.fill_diagonal(self.R, [0.5,0.5,0.5,0.01,0.01,0.01])
        self.n = 6                                            #Number of independant state variabls
        self.thld = 1e-5
        self.MaxIter = 2000


    def quat_to_euler(self,q):
        w,x,y,z = np.float64(q)
        R11 = 1-2*(y**2 +z**2)
        R21 = 2*(w*z + x*y)
        R31 = 2*(-x*z+w*y)
        R32 = 2*(y*z+w*x)
        R33 = 2*(w**2)-1+2*(z**2)
        if R31**2 >=1.0:
            # print "true"
            R31 = 0.99499
        
        phi = math.atan2(R32, R33 )*(180/np.pi);
        theta = math.atan(R31/math.sqrt(1-R31**2) )*(180/np.pi);
        psi = math.atan2(R21, R11 )*(180/np.pi);
        return phi, theta, psi
        
    def quaternion_multiply(self,quaternion1, quaternion0):

        w0, x0, y0, z0 = np.float64(quaternion1)
        w1, x1, y1, z1 = np.float64(quaternion0)
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     	  x1 * w0 - y1 * z0 + z1 * y0 + w1 * x0,
                          x1 * z0 + y1 * w0 - z1 * x0 + w1 * y0,
                     	 -x1 * y0 + y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    def quat2RV(self,q):
		sinalpha = np.linalg.norm(q[1:4])
		cosalpha = q[0]

		alpha = math.atan2(sinalpha,cosalpha)
		if (sinalpha==0):
			rv = np.array([0,0,0],dtype=np.float64)
			return rv

		e = q[1:4]/float(sinalpha)
		rv = e*2.*alpha
		return rv


    def qfromEuler(self,eul_angles,dtt=1.0):
        norm = np.linalg.norm(eul_angles)
        # print(norm)
        dtt = dtt
        if norm==0:
            q = np.array([1,0,0,0],dtype=np.float64)
            return q
        alpha = norm*dtt
        ew = eul_angles*(dtt/alpha)
        q = np.array([math.cos(alpha/2),
                      ew[0]*math.sin(alpha/2),
                      ew[1]*math.sin(alpha/2),
                      ew[2]*math.sin(alpha/2)])
        q = q/np.linalg.norm(q)
        return q
    
    def get_sig_pts(self):
		num_pts = self.W.shape[1]
		X1 = np.zeros((4,12))
		for pt in range(num_pts):
		    qw = self.qfromEuler(self.W[:,pt])
		    X1[:,pt] = self.quaternion_multiply(self.state[:-3],qw)
		X2 = self.W[-3:] + self.state[-3:, np.newaxis]
		sigma_pts = np.concatenate([X1,X2],axis=0)
		return sigma_pts

    def get_tfm_sig_pts(self,X):
    	#Returns quaternion part of sigma points and transformed sigma points
    	# Input = state update transformed into quaternion
    	# Output X1(sigma points) and Y1(transformed sigma points)
		num_pts = self.W.shape[1]
		Y1 = np.zeros((4,12))
		# print("q from w = ",self.state[:-3], np.linalg.norm(self.state[:-3]))
		# input('awev')
		for pt in range(num_pts):
		    
		    omega = X[-3:,pt]
		    delta_q = self.qfromEuler(omega,self.dt)
		    # qw = self.quaternion_multiply(qw,delta_q)
		    Y1[:,pt] = self.quaternion_multiply(X[:-3,pt],delta_q)
		Y2 = X[-3:,:]
		Tfm_sigma_pts = np.concatenate([Y1,Y2],axis=0)
		return Tfm_sigma_pts

    def qinv(self,q):
        w,x,y,z = q
        q_conj = np.array([w,-x,-y,-z])
        q_inv = q_conj/(np.linalg.norm(q))
        return q_inv
    
    def Gen_sigma_points(self):    
        self.S = np.linalg.cholesky(self.P + self.Q)                   #cholski square root
        # print(self.S)
        # input('a')
        
        self.W = np.concatenate([np.sqrt(self.n)* self.S,
                                 -np.sqrt(self.n) * self.S],axis=1)

        # print(self.W)
        # input('aaaa')
        #Process model
        # omega = self.state[-3:]
        # print(self.state)
        # delta_q = self.qfromEuler(omega,self.dt)
        # print(delta_q)
        # input('aaaa')
        X = self.get_sig_pts()
        Y = self.get_tfm_sig_pts(X)
        # X1,Y1 = self.getX1Y1fromW()                           #Directly get updated sigma points
        # X2 = self.W[-3:] + self.state[-3:, np.newaxis]
        # # print(X2)
        # # input('a')
        # sigma_pts = np.concatenate([X1,X2],axis=0)
        

        return X,Y

    def intrinsicGradientDescent(self, sigma_pts, Tfm_sigma_pts):
        # print(sigma_pts)
        # input('sigma_pts')
        qbar = sigma_pts[:-3]
        # print("qbar init = ", qbar)
        # input('aaa')
        num_pts = Tfm_sigma_pts.shape[1]
        err_vectors =np.zeros((num_pts,3))  #12x3
        iter_ = 1
        mean_err = np.array([10000.,10000.,10000.])
        while(np.linalg.norm(mean_err)>self.thld and iter_<=self.MaxIter):
            for i in range(num_pts):
                qi = Tfm_sigma_pts[:-3,i]
                # print(qi)
                # input('qi')
                err_quat = self.quaternion_multiply(qi,self.qinv(qbar))
                # print(err_quat)
                # input('err_quat')
                err_vectors[i,:] = self.quat2RV(err_quat)
                # print(err_vectors[i])
                # input('err_vectors')
            # compute mean of all the err_vectors
            mean_err = np.mean(err_vectors,axis = 0)
            # print(mean_err)
            # convert mean error rotation vector to quaternion
            mean_err_quat = self.qfromEuler(mean_err)
            qbar = self.quaternion_multiply(mean_err_quat,qbar)
            qbar = np.divide(qbar,np.linalg.norm(qbar))
            iter_ += 1

        # print Tfm_sigma_pts[-3:,:]
        # input('A')
        omega_bar = (1/float(num_pts))*np.mean(Tfm_sigma_pts[-3:,:], axis=1)
        # print("qbar",qbar)
        # # print("sigma_pts", sigma_pts[-3:,:])
        # print("omega_bar",omega_bar)
        # input('a')
        return qbar,omega_bar   #xkbar

    # def Compute_transformed_sigma_pts(X,dt):
    #     Y = np.zeros_like(X)
    #     num_pts = X.shape()[1]
    #     for i in range(num_pts):
    #         quat = qfromEuler(X[:4,i])

    def get_W_dash(self, Y,qbar,omega_bar):
        num_pts = Y.shape[1]
        W_dash_quat = np.zeros((3,num_pts))
        for i in range(num_pts):
            quat = self.quaternion_multiply(self.qinv(qbar),Y[:4,i])
            W_dash_quat[:,i] = self.quat2RV(quat)

        # check if subtraction of 3xn and 3x1 works this way...
        W_dash_omega = self.W[-3:,:]-omega_bar[-3:,np.newaxis]
        # print(W_dash_omega)
        # input('a')
        W_dash = np.concatenate([W_dash_quat,W_dash_omega],axis=0)

        return W_dash

    def update_model_cov(self, W_dash):
        num_pts = float(W_dash.shape[1])
        Pk_bar = np.float64((1/(num_pts))*np.matmul(W_dash,W_dash.T))
        # print(Pk_bar[0,0])
        return Pk_bar

    def compute_Z(self, Y):
        num_pts = Y.shape[1]
        # Gravity vector in quaternion
        g = np.array([0,0,0,1],dtype=np.float64)
        Z = np.zeros((6,num_pts))
        # print("Y = ",Y)
        # input("a")
        for i in range(num_pts):
            quat = self.quaternion_multiply(self.qinv(Y[:-3,i]),g)
            # print("quat 1 ",quat)
            quat = self.quaternion_multiply(quat,Y[:-3,i])
            # print("quat ",quat)
            # input("quat")
            Z[:,i] = np.concatenate([self.quat2RV(quat),Y[-3:,i]],axis=0)
        # print(self.quat2RV(quat))
        # input('Z')
        return Z

    def compute_Z_mean(self,Z):
        return np.mean(Z,axis=1)

    def Z_mean_centered(self, Z):
        # center the Z around mean
        Z_mean = self.compute_Z_mean(Z)
        # print("Z_mean = ",Z_mean)
        # input("qas")
        Z_centered = Z - Z_mean[:,np.newaxis]
        return Z_centered

    def Compute_vk(self, acc, gyro, Z):
        #compute the innovation term vk
        Zk = np.concatenate((acc,gyro),axis=0)
        # print("zk = ",Zk)
        # input("zk")
        # print("Z, zk", len(self.compute_Z_mean(Z)), Zk.shape)
        vk = Zk - self.compute_Z_mean(Z)
        # print(self.compute_Z_mean(Z))
        # input('sw')
        return vk

    def Compute_Inn_cov(self, Z_mean_centered):
        # Compute Innovation covariance
        num_pts = float(Z_mean_centered.shape[1])
        Pzz = (1/num_pts)*np.matmul(Z_mean_centered,Z_mean_centered.T)
        Pvv = Pzz + self.R
        return Pvv

    def Compute_cross_corr_mat(self, W_dash, Z_mean_centered):
        num_pts = float(Z_mean_centered.shape[1])
        Pxz = (1/(num_pts))*np.matmul(W_dash,Z_mean_centered.T)
        return Pxz

    def State_update(self,Pxz,Pvv,Vk,xkbar,Pk_bar):
        #Compute Kalman Gain and update state
        Kk = np.matmul(Pxz,np.linalg.inv(Pvv))
        # print("kk",np.float16(Kk))
        # input("kk")
        # xk = xkbar + np.matmul(Kk,Vk)
        Kkvk = np.matmul(Kk,Vk)
        # print("kkvk",np.float16(Kkvk))
        # input("kkvk")
        quat = self.quaternion_multiply(xkbar[:4],self.qfromEuler(Kkvk[:3]))
        quat = np.divide(quat,np.linalg.norm(quat))
        # print("xkbar = ",xkbar[-3:],Kkvk[-3:])
        xkbarhat = np.concatenate((quat,xkbar[-3:]+Kkvk[-3:]),axis=0)
        # print("xkbarhat",np.float16(xkbarhat))
        # input("kkvk")
        # 
        Pk = Pk_bar - np.matmul(Kk,np.matmul(Pvv,Kk.T))
        # print xkbarhat
        # input('a')
        return Pk, xkbarhat

    def RunUkf(self, acc, gyro,dt=0.01):
		# Compute transformed sigma pts
		self.dt = dt
		X,Y = self.Gen_sigma_points()
		# print(Y)
		# input('a')
		# Get mean value of Y; xkbar = [qbar,omega_bar]
		qbar,omega_bar = self.intrinsicGradientDescent(X[:,0],Y)
		# print("qbar = ", qbar)
		# print("omega_bar/// = ", omega_bar)
		# input('aaaaa')
		xkbar = np.concatenate((qbar,omega_bar),axis=0)
		# print ("xkbar = ",xkbar)
		# input('a')

		W_dash = self.get_W_dash(Y,qbar,omega_bar)
		# print("X", X)
		# input("A")
		#  Compute process model covariance
		Pk_bar = self.update_model_cov(W_dash)
		# print("Pk_bar",np.float16(Pk_bar))
		# input("kkvk")
		# Apply the measurement model 
		Z = self.compute_Z(Y)

		# print("Z = ",Z)
		# input("X")
		# print("Y = ",Y)
		# input("Y")
		# print("Z = ",Z)
		# input("Z")
		# Center Z around mean 
		Z_centered = self.Z_mean_centered(Z)
		# print("Z_mean_centered = ",Z_centered)
		# input("SA")
		# Compute innovation term
		# print("acc and gyro",acc, gyro)
		# input("acc")
		vk = self.Compute_vk(acc, gyro, Z)
		# print("vk = ",vk)
		# input("vk")
		# Compute innovation covariances
		Pvv = self.Compute_Inn_cov(Z_centered)
		# print("Pvv = ",np.float16(Pvv))
		# input("pvv")
		# Compute Cross correlation matrix
		Pxz = self.Compute_cross_corr_mat(W_dash, Z_centered)
		# print("Pxz = ",np.float16(Pxz))
		# input("pxz")
		# Update state

		# Update covariance
		Pk,xkbarhat = self.State_update(Pxz,Pvv,vk,xkbar,Pk_bar)
		# print("Pk",np.float16(Pk))
		# input("kkvk")

		self.state = xkbarhat
		self.P = Pk

		return xkbarhat, Pk


def process_data(acc_data,acc_scale,acc_bias,gyro):

    acc_data = np.multiply(acc_data,acc_scale[:,np.newaxis])
    acc_data = acc_data + acc_bias[:,np.newaxis]

    gyro_bias = np.mean(gyro[:,:200],axis = 1)

    gyro_data = gyro - gyro_bias[:,np.newaxis]
    gyro_data = (3300/1023)*(np.pi/180)*(1/3.33)*gyro_data[[1,2,0],:]

    return acc_data, gyro_data

# def rotvec_to_euler(R):
# #     w,x,y,z = q
# #     roll  = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
# #     pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
# #     yaw = math.asin (2*x*y + 2*z*w)
#     R11 = R[0,0]
#     R21 = R[1,0]
#     R31 = R[2,0]
#     R32 = R[2,1]
#     R33 = R[2,2]
    
#     phi = math.atan2(R32, R33 )*(180/np.pi);
#     theta = -math.atan(R31/math.sqrt(1-R31**2) )*(180/np.pi);
#     psi = math.atan2(R21, R11 )*(180/np.pi);
#     return np.array((phi, theta, psi))


def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))


def main():
	ukf = UKF()
	xkbarhat_arr = []
	# read acc and gyro data
	data = loadmat('../drone_course_data/Data/Train/IMU/imuRaw1.mat')
	vicon_data = loadmat('../drone_course_data/Data/Train/Vicon/viconRot1.mat')
	# vicon_data = ' '
	imu_params = loadmat('../drone_course_data/Data/IMUParams.mat')
	acc_scale = imu_params['IMUParams'][0]
	acc_bias = imu_params['IMUParams'][1]
	ts = data['ts']

	acc_data = data['vals'][:3,:]
	gyro_data = data['vals'][-3:,:]
	num_iter = acc_data.shape[1]
	# num_iter = 2000
	start = 0
	print num_iter
	count = 0
	# time2 = np.arange(1,vicon_data['rots'].shape[2]+1)
	time2 = [x+1 for x in range(num_iter)]#np.arange(1,num_iter+1)
	if vicon_data is not " " :
		rots = vicon_data['rots']
	align_imu = [0]*40
	num_vic = 1
	align_vicon = [0]*num_vic
	rots_dataX = []
	rots_dataY = []
	rots_dataZ = []
	eul_ukf_X = []
	eul_ukf_Y = []
	eul_ukf_Z = []
	eul_mdw_X = []
	eul_mdw_Y = []
	eul_mdw_Z = []
	time = np.arange(1,acc_data.shape[1]+1)
	acc_data, gyro_data = process_data(acc_data,acc_scale,acc_bias,gyro_data)
	q_init = Quaternion(1,0,0,0)
	q_est = Quaternion(1,0,0,0)
	mdw = madgwick(q = q_init)
	pbar = tqdm.tqdm(total = num_iter)
	# print (acc_data[:,0], gyro_data[:,0])
	# input('a')
	while count<num_iter:
		if count == 0:
			dt = 0.01
		else:
			dt = ts[0][count]-ts[0][count-1]
		xkbarhat,Pk = ukf.RunUkf(acc_data[:,count],gyro_data[:,count],dt)
		q,temp1, temp2 = mdw.imu_update(acc_data[:,count].reshape(3,1),gyro_data[:,count].reshape(3,1),q_est)
		eulX, eulY, eulZ = ukf.quat_to_euler(q) 
		q_est = Quaternion(q[0],q[1],q[2],q[3])
		# if count>1000:
		# 	print("xkbarhat = ", xkbarhat)
		# 	print("pk = ", Pk)
		# 	input("asa")
		xkbarhat_arr.append(xkbarhat)
		eul_angles = ukf.quat_to_euler(xkbarhat[:4])
		# print("euler angles = ",eul_angles)
		#  Converting Vicon data to euler angles
		try:
			rr = rot2eul(rots[:,:,start+count])*(180/np.pi)

			rots_dataX.append(rr[0])
			rots_dataY.append(rr[1])
			rots_dataZ.append(rr[2])
			
		except:
			pass

		# print ("rr = ",rr)
		eul_ukf_X.append(eul_angles[0])
		eul_ukf_Y.append(eul_angles[1])
		eul_ukf_Z.append(eul_angles[2])
		eul_mdw_X.append(eulX)
		eul_mdw_Y.append(eulY)
		eul_mdw_Z.append(eulZ)
		pbar.update(1)
		count+=1
	pbar.close()
	# print np.shape(rots_data)
	# plt.plot(time2, acc[0,start:start+num_iter],'b')
	# plt.plot(time2, gyro[0,start:start+num_iter],'r')
	
	fig=plt.figure(1)
	time_rot = [x+1 for x in range(len(rots_dataX)+num_vic)]
	
	# input("sad")
	# plt.plot(time, acc_data[0,:],'b')
	# plt.plot(time, gyro_data[0,:],'r')
	# plt.plot(time2, rots_dataX,'-k')
	# plt.plot(time2, eul_all,'c')

	# print(align_vicon+time2, len(align_imu+time2), len(align_imu+eul_mdw_X), len(align_imu))
	a1 = plt.subplot(4,1,1)
	# line1, = a1.plot(time,acc_data[0,:],'y')
	# line1.set_label('acc data')
	line2, = a1.plot(align_imu+time2,align_imu+eul_mdw_X,'g')
	if vicon_data is not ' ':
		line3, = a1.plot(time_rot,align_vicon+rots_dataX,'b')
		line3.set_label('x Vicon')
	line4, = a1.plot(align_imu+time2,align_imu+eul_ukf_X,'r')
	# line1.set_label('x acc')
	line2.set_label('x madgwick')
	line4.set_label('x UKF')

	a1.title.set_text('X axis')
	a1.legend()


	a2 = plt.subplot(4,1,2)
	# line5, = a2.plot(time,acc_data[1,:],'y')
	# line1.set_label('acc data')
	line6, = a2.plot(align_imu+time2,align_imu+eul_mdw_Y,'g')
	if vicon_data is not ' ':
		line7, = a2.plot(time_rot,align_vicon+rots_dataY,'b')
		line7.set_label('y Vicon')
	line8, = a2.plot(align_imu+time2,align_imu+eul_ukf_Y,'r')
	# line5.set_label('y acc')
	line6.set_label('y madgwick')
	line8.set_label('y UKF')

	a2.title.set_text('Y axis')
	a2.legend()


	a3 = plt.subplot(4,1,3)
	# line9, = a3.plot(time,acc_data[2,:],'y')
	# line1.set_label('acc data')
	line10, = a3.plot(align_imu+time2,align_imu+eul_mdw_Z,'g')
	if vicon_data is not ' ':
		line11, = a3.plot(time_rot,align_vicon+rots_dataZ,'b')
		line11.set_label('z Vicon')
	line12, = a3.plot(align_imu+time2,align_imu+eul_ukf_Z,'r')
	# line9.set_label('z acc')
	line10.set_label('z madgwick')
	line12.set_label('z UKF')

	a3.title.set_text('Z axis')
	a3.legend()

	plt.show()
        
if __name__ == '__main__':
    main()