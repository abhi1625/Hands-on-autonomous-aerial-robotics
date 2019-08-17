import numpy as np
import math
# import pyquaternion as quat

class UKF():
    def __init__(init_state=np.array([1,0,0,0,0,0,0]), P0 = np.zeros((6,6))):
        self.dt
        self.state = init_state                               #initial state
        self.P = P0                                           #initial covariance matrix
        self.Q                                                #Process Noise
        self.R                                                #Measurement Noise
        self.n = 6                                            #Number of independant state variabls
        
    def quaternion_multiply(self,quaternion1, quaternion0):

        w0, x0, y0, z0 = quaternion1
        w1, x1, y1, z1 = quaternion0
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    def quat2RV(self,q):
        w,x,y,z = q
        thetha = 2*math.acos(w)

        b,c,d = (thetha/math.sin(thetha/2))*np.array([x,y,z])

        return np.array([b,c,d])

    def qfromEuler(eul_angles):
        norm = np.linalg.norm(eul_angles)
        q = np.array([math.cos(norm*self.dt/2),
                      eul_angles[0]*math.sin(norm*self.dt/2)/norm,
                      eul_angles[1]*math.sin(norm*self.dt/2)/norm,
                      eul_angles[2]*math.sin(norm*self.dt/2)/norm])
        return q
    
    def qfromW(self, delta_q):
        num_pts = self.W.get_shape()[1]
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
        self.S = np.linalg.cholesky(P + Q)                   #cholski square root
        
        self.W = np.concatenate([math.sqrt(2*self.n * self.S),
                                 -math.sqrt(2*self.n * self.S)],axis=1)

        #Process model
        omega = self.state[-3:]
        delta_q = self.qfromEuler(omega)

        X1 = self.qfromW(delta_q)                           #Directly get updated sigma points
        X2 = self.W[-3:] + self.state[-3:, np.newaxis]

        sigms_pts = np.concatenate([X1,X2],axis=0)

        return sigma_pts

    def intrinsicGradientDescent(self, sigma_pts, thld, MaxIter):
        qbar = sigma_pts[:-3,0]
        num_pts = sigma_pts.shape()[1]
        err_vectors =np.zeros((num_pts,3))  #12x3
        iter_ = 1
        while(np.linalg.norm(mean_err)<thld and iter_<=MaxIter):
            for i in range(num_pts):
                qi = sigma_pts[:-3,i]
                err_quat = self.quaternion_multiply(qi,self.qinv(qbar))
                err_vectors[i,:] = self.quat2RV(err_quat)

            # compute mean of all the err_vectors
            mean_err = np.mean(err_vectors,axis = 0)
            # convert mean error rotation vector to quaternion
            mean_err_quat = self.qfromEuler(mean_err)
            qbar = self.quaternion_multiply(mean_err_quat,qbar)
            iter_ += 1


        omega_bar = (1/(2*num_pts))*np.mean(sigma_pts[-4:,:], axis=0)
        return qbar,omega_bar   #xkbar

    def get_W_dash(self, Y,qbar,omega_bar):
        num_pts = Y.shape()[1]
        W_dash_quat = np.zeros((3,num_pts))
        for i in range(num_pts):
            quat = self.quaternion_multiply(Y(:4,i),self.qinv(qbar))
            W_dash_quat[:,i] = self.quat2RV(quat)

        W_dash_omega = self.W(-3:,:)*omega_bar
        
        W_dash = np.concatenate([W_dash_quat,W_dash_omega],axis=0)

        return W_dash

    def update_model_cov(self, W_dash):
        num_pts = W_dash.shape()[1]
        Pk_bar = (1/(2*num_pts))*np.matmul(W_dash,W_dash.T)
        return Pk_bar

    def compute_Z(self, Y):
        num_pts = Y.shape()[1]
        # Gravity vector in quaternion
        g = np.array([0,0,0,1])
        Z = np.zeros((6,num_pts))
        
        for i in range(num_pts):
            quat = self.quaternion_multiply(self.qinv(Y[:-3,i]),g)
            quat = self.quaternion_multiply(quat,Y[:-3,i])
            Z[:,i] = np.concatenate([quat2RV(quat),Y[-3:,i]],axis=0)
        return Z

    def compute_z_mean(self,Z):
        return np.mean(Z,axis=1)

    def Z_mean_centered(self, Z):
        Z_mean = compute_z_mean(Z)
        Z_centered = Z - Z_mean
        return Z_centered

    def Compute_vk(self, acc, gyro):
        #compute the innovation term vk
         

        return






            
        
        
