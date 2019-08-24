import numpy as np
import math
from scipy.io import loadmat
import matplotlib.pyplot as plt
# import pyquaternion as quat

class UKF:
    def __init__(self,init_state=np.array([1,0,0,0,0,0,0]), P0 = (1e-2)*np.zeros((6,6))):

        self.dt = 0.01
        self.state = init_state                               #initial state
        self.P = P0                                           #initial covariance matrix
        # Process Model noise
        self.Q = np.zeros((6,6))
        np.fill_diagonal(self.Q, [100,100,100,0.1,0.1,0.1])
        # Measurement Model noise
        self.R = np.zeros((6,6))
        np.fill_diagonal(self.R, [0.5,0.5,0.5,0.001,0.001,0.001])
        self.n = 6                                            #Number of independant state variabls
        self.thld = 1e-5
        self.MaxIter = 1000
        
    def quaternion_multiply(self,quaternion1, quaternion0):

        w0, x0, y0, z0 = quaternion1
        w1, x1, y1, z1 = quaternion0
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    def quat2RV(self,q):
        q = np.divide(q,np.linalg.norm(q))
        w,x,y,z = q

        thetha = 2*math.acos(w)
        if thetha==0:
            b,c,d = np.zeros(3,) 
        else:
            b,c,d = (thetha/math.sin(thetha/2))*np.array([x,y,z])

        return np.array([b,c,d])

    def qfromEuler(self,eul_angles):
        norm = np.linalg.norm(eul_angles)
        # print(norm)
        if norm==0:
            q = np.array([1,0,0,0])
            return q

        q = np.array([math.cos(norm*self.dt/2),
                      eul_angles[0]*math.sin(norm*self.dt/2)/norm,
                      eul_angles[1]*math.sin(norm*self.dt/2)/norm,
                      eul_angles[2]*math.sin(norm*self.dt/2)/norm])
        return q
    
    def qfromW(self, delta_q):
        num_pts = self.W.shape[1]
        X1 = np.zeros((4,12))
        for pt in range(num_pts):
            qw = self.qfromEuler(self.W[:,pt])

            #process model update
            qw = self.quaternion_multiply(qw,delta_q)
            X1[:,pt] = self.quaternion_multiply(self.state[:-3],qw)

        return X1

    def qinv(self,q):
        w,x,y,z = q
        q_conj = np.array([w,-x,-y,-z])
        q_inv = q_conj/(np.linalg.norm(q))
        return q_inv
    
    def Gen_sigma_points(self):    
        self.S = np.linalg.cholesky(self.P + self.Q)                   #cholski square root
        # print(self.S)
        # input('a')
        
        self.W = np.concatenate([np.sqrt(2*self.n * self.S),
                                 -np.sqrt(2*self.n * self.S)],axis=1)

        # print(self.W)
        # input('aaaa')
        #Process model
        omega = self.state[-3:]
        # print(omega)
        delta_q = self.qfromEuler(omega)
        # print(delta_q)
        # input('aaaa')
        X1 = self.qfromW(delta_q)                           #Directly get updated sigma points
        X2 = self.W[-3:] + self.state[-3:, np.newaxis]

        sigma_pts = np.concatenate([X1,X2],axis=0)

        return sigma_pts

    def intrinsicGradientDescent(self, sigma_pts):
        # print(sigma_pts)
        # input('sigma_pts')
        qbar = sigma_pts[:-3,0]

        num_pts = sigma_pts.shape[1]
        err_vectors =np.zeros((num_pts,3))  #12x3
        iter_ = 1
        mean_err = np.array([10000,10000,10000])
        while(np.linalg.norm(mean_err)>self.thld and iter_<=self.MaxIter):
            for i in range(num_pts):
                qi = sigma_pts[:-3,i]
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


        omega_bar = (1/(2*num_pts))*np.mean(sigma_pts[-3:,:], axis=1)
        print("qbar",qbar)
        print("sigma_pts", sigma_pts[-3:,:])
        print("omega_bar",omega_bar)
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
            quat = self.quaternion_multiply(Y[:4,i],self.qinv(qbar))
            W_dash_quat[:,i] = self.quat2RV(quat)

        # check if subtraction of 3xn and 3x1 works this way...
        W_dash_omega = self.W[-3:,:]-omega_bar[-3:,np.newaxis]
        
        W_dash = np.concatenate([W_dash_quat,W_dash_omega],axis=0)

        return W_dash

    def update_model_cov(self, W_dash):
        num_pts = W_dash.shape[1]
        Pk_bar = (1/(2*num_pts))*np.matmul(W_dash,W_dash.T)
        return Pk_bar

    def compute_Z(self, Y):
        num_pts = Y.shape[1]
        # Gravity vector in quaternion
        g = np.array([0,0,0,1])
        Z = np.zeros((6,num_pts))
        
        for i in range(num_pts):
            quat = self.quaternion_multiply(self.qinv(Y[:-3,i]),g)
            quat = self.quaternion_multiply(quat,Y[:-3,i])
            Z[:,i] = np.concatenate([self.quat2RV(quat),Y[-3:,i]],axis=0)
        return Z

    def compute_Z_mean(self,Z):
        return np.mean(Z,axis=1)

    def Z_mean_centered(self, Z):
        # center the Z around mean
        Z_mean = self.compute_Z_mean(Z)
        Z_centered = Z - Z_mean[:,np.newaxis]
        return Z_centered

    def Compute_vk(self, acc, gyro, Z):
        #compute the innovation term vk
        Zk = np.concatenate((acc,gyro),axis=0)
        vk = Zk - self.compute_Z_mean(Z)
        return vk

    def Compute_Inn_cov(self, Z_mean_centered):
        # Compute Innovation covariance
        num_pts = Z_mean_centered.shape[1]
        Pzz = (1/(2*num_pts))*np.matmul(Z_mean_centered,Z_mean_centered.T)
        Pvv = Pzz + self.R
        return Pvv

    def Compute_cross_corr_mat(self, W_dash, Z_mean_centered):
        num_pts = Z_mean_centered.shape[1]
        Pxz = (1/(2*num_pts))*np.matmul(W_dash,Z_mean_centered.T)
        return Pxz

    def State_update(self,Pxz,Pvv,Vk,xkbar,Pk_bar):
        #Compute Kalman Gain and update state
        Kk = np.matmul(Pxz,np.linalg.inv(Pvv))
        # 
        # xk = xkbar + np.matmul(Kk,Vk)
        Kkvk = np.matmul(Kk,Vk)
        quat = self.quaternion_multiply(xkbar[:4],self.qfromEuler(Kkvk[:3]))
        quat = np.divide(quat,np.linalg.norm(quat))
        # print("xkbar = ",xkbar[-3:],Kkvk[-3:])
        xkbarhat = np.concatenate((quat,xkbar[-3:]+Kkvk[-3:]),axis=0)

        # 
        Pk = Pk_bar - np.matmul(Kk,np.matmul(Pvv,Kk.T))
        return Pk, xkbarhat

    def RunUkf(self, acc, gyro):
        # Compute transformed sigma pts
        Y = self.Gen_sigma_points()
        # Get mean value of Y; xkbar = [qbar,omega_bar]
        qbar,omega_bar = self.intrinsicGradientDescent(Y)
        # print("qbar = ", qbar)
        # print("omega_bar = ", omega_bar)
        # input('aaaaa')
        xkbar = np.concatenate((qbar,omega_bar),axis=0)
        # print ("xkbar = ",xkbar)
        
        W_dash = self.get_W_dash(Y,qbar,omega_bar)
        #  Compute process model covariance
        Pk_bar = self.update_model_cov(W_dash)

        # Apply the measurement model 
        Z = self.compute_Z(Y)

        # Center Z around mean 
        Z_centered = self.Z_mean_centered(Z)

        # Compute innovation term
        vk = self.Compute_vk(acc, gyro, Z)

        # Compute innovation covariances
        Pvv = self.Compute_Inn_cov(Z_centered)

        # Compute Cross correlation matrix
        Pxz = self.Compute_cross_corr_mat(W_dash, Z_centered)

        # Update state

        # Update covariance
        Pk,xkbarhat = self.State_update(Pxz,Pvv,vk,xkbar,Pk_bar)

        self.state = xkbarhat
        self.P = Pk

        return xkbarhat, Pk


def rotvec_to_euler(R):
#     w,x,y,z = q
#     roll  = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
#     pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
#     yaw = math.asin (2*x*y + 2*z*w)
    R11 = R[0,0]
    R21 = R[1,0]
    R31 = R[2,0]
    R32 = R[2,1]
    R33 = R[2,2]
    
    phi = math.atan2(R32, R33 )*(180/np.pi);
    theta = -math.atan(R31/math.sqrt(1-R31**2) )*(180/np.pi);
    psi = math.atan2(R21, R11 )*(180/np.pi);
    return phi, theta, psi

def main():
    num_iter = 100
    count = 0
    ukf = UKF()
    start = 800
    xkbarhat_arr = []
    # read acc and gyro data
    data = loadmat('../../drone_course_data/UKF/imu/imuRaw2.mat')
    vicon_data = loadmat('../../drone_course_data/UKF/vicon/viconRot2.mat')
    imu_params = loadmat('/home/pratique/git_cloned_random/ESE650Project2-master/Preprocess/IMUParams.mat')
    acc_scale = imu_params['IMUParams'][0]
    acc_bias = imu_params['IMUParams'][1]

    print(acc_scale)
    input('a')
    acc = data['vals'][:3,:]
    # acc =  np.matmul(acc_scale[np.newaxis,:],acc)
    print(acc.shape)
    input('a')
    gyro = data['vals'][-3:,:]
    # time2 = np.arange(1,vicon_data['rots'].shape[2]+1)
    time2 = np.arange(1,num_iter+1)
    rots = vicon_data['rots']
    rots_dataX = []
    rots_dataY = []
    rots_dataZ = []
    eul_all = []
    time = np.arange(1,acc.shape[1]+1)

    while count<num_iter:
        acc_data = np.multiply(acc_scale[:,np.newaxis],acc[:,start+count][:,np.newaxis])
        acc_data = acc_data + acc_bias[:,np.newaxis]
        acc_data = acc_data[:,0]
        gyro_data = 3300/1023*np.pi/180*(1/3.33)*gyro[:,start+count]
        xkbarhat,Pk = ukf.RunUkf(acc_data,gyro_data)
        xkbarhat_arr.append(xkbarhat)
        eul_angles = ukf.quat2RV(xkbarhat[:4])
        print("euler angles = ",eul_angles)
        rr = np.array(rotvec_to_euler(rots[:,:,start+count]))
        # print ("rr = ",rr)
        eul_all.append(eul_angles[0])
        rots_dataX.append(rr[0])
        rots_dataY.append(rr[1])
        rots_dataZ.append(rr[2])
        count+=1
    # print np.shape(rots_data)
    # plt.plot(time2, acc[0,start:start+num_iter],'b')
    # plt.plot(time2, gyro[0,start:start+num_iter],'r')
    plt.plot(time, acc[0,:],'b')
    plt.plot(time, gyro[0,:],'r')
    plt.plot(time2, rots_dataX,'-k')
    plt.plot(time2, eul_all,'c')
    plt.show()

        
if __name__ == '__main__':
    main()