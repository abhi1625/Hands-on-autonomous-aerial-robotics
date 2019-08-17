import numpy as np
import math
import pyquaternion as quat

class UKF():
    def __init__(init_state=np.array([1,0,0,0,0,0,0]), P0 = np.zeros(6,6)):
        self.dt
        self.state = init_state                               #initial state
        self.P = P0                                           #initial covariance matrix
        self.Q                                                #Process Noise
        self.R                                                #Measurement Noise
        self.n = 6                                            #Number of independant state variabls
        
    def quaternion_multiply(self,quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    def qfromEuler(eul_angles):
        norm = np.linalg.norm(eul_angles)
        q = np.array([math.cos(norm*self.dt/2),
                      eul_angles[0]*math.sin(norm*self.dt/2)/norm,
                      eul_angles[1]*math.sin(norm*self.dt/2)/norm,
                      eul_angles[2]*math.sin(norm*self.dt/2)/norm])
        return q
    
    def qfromW(self, delta_q):
        num_pts = self.W.get_shape()[1]
        X1 = np.zeros(4,12)
        for pt in range(num_pts):
            qw = self.qfromEuler(self.W[:,pt])

            #process model update
            qw = self.quaternion_multiply(delta_q, qw)
            X1[:,pt] = self.quaternion_multiply(qw, self.state[:-3])

        return X1
    
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

    def intrinsicGradientDescent(self, sigma_pts, qbar, thld, MaxIter):
        num_pts = sigma_pts.shape()[1]
        err_vectors =np.zeros(num_pts,1) 
        for i in range():
            qi = sigma_pts(:-3)[:,i]
            err_vectors[i] = self.quaternion_multiply(qinv(qbar), qi)

            
        
        
