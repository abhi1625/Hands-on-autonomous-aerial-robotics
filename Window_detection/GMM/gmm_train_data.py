import numpy as np
import cv2
import math
import os
import matplotlib.pyplot as plt


def save_models(weights, weights_name, parameters, params_name):
    np.save(weights_name, weights)
    np.save(params_name, parameters)


def getData(folder_name):
    stack = []
    for filename in sorted(os.listdir(folder_name)):
        
        image = cv2.imread(os.path.join(folder_name, filename))
        thresh = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # resized = cv2.resize(image, (40, 40), interpolation=cv2.INTER_LINEAR)
        #data to be stored as RGB

        # image = resized[13:27, 13:27]
        image = image[:,:,[2,1,0]]

        ch = image.shape[2]
        nx = image.shape[0]
        ny = image.shape[1]
        image = np.reshape(image, (nx*ny, ch))
        thresh = np.reshape(thresh,(nx*ny,1))
        #print(image.shape)
        for i in range(image.shape[0]):
            if ((thresh[i,:]<100) or (thresh[i,:]>225)):
                # print "zero h"
                continue
            stack.append(image[i, :])

        print("frame processed")
        print("length of data stack: ", len(stack))
    
    np.save('yellow_window_data_100.npy',stack)
    return np.array(stack)

class GMM:
    def __init__(self,data):
        self.data = data

    def gaussian(self, x, mean, cov,n_obs):
        
        det_cov = np.linalg.det(cov)
        cov_inv = np.zeros_like(cov)
        for i in range(n_obs):
            cov_inv[i, i] = 1/cov[i, i]
        #cov_inv = np.linalg.inv(cov)
        diff = np.matrix(x-mean)
        
        N = (2.0 * np.pi) ** (-len(self.data[1]) / 2.0) * (1.0 / (np.linalg.det(cov) ** 0.5)) *\
            np.exp(-0.5 * np.sum(np.multiply(diff*cov_inv, diff), axis=1))
        return N


    def initialize(self, n_feat, n_obs):
        mean = np.array([self.data[np.random.choice(n_feat, 1)]], np.float64)
        cov = [np.random.randint(1, 255)*np.eye(n_obs)]
        #print(cov)
        cov = np.matrix(np.multiply(cov,np.random.rand(n_obs, n_obs)))
        #print(cov)
        return {'mean': mean, 'cov': cov}

    def GMM(self, K):
        
        n_feat = self.data.shape[0] 
        n_obs = self.data.shape[1]


        bound = 0.0001
        max_itr = 500
        
        parameters = [self.initialize(n_feat, n_obs) for cluster in range (K)]
        cluster_prob = np.ndarray([n_feat, K], np.float64)
        
        #EM - step E
        itr = 0
        mix_c = [1./K]*K
        log_likelihoods = []
        while (itr < max_itr):
            # print(itr)
            itr+=1

            for cluster in range (K):
                cluster_prob[:, cluster:cluster+1] = self.gaussian(self.data, parameters[cluster]['mean'], parameters[cluster]['cov'],n_obs)*mix_c[cluster]

            cluster_sum = np.sum(cluster_prob, axis=1)
            log_likelihood = np.sum(np.log(cluster_sum))
            
            log_likelihoods.append(log_likelihood)
            #print(log_likelihoods)
            cluster_prob = np.divide(cluster_prob,np.tile(cluster_sum,(K,1)).transpose())
            
            Nk = np.sum(cluster_prob,axis = 0) #2

            #EM - step M
            for cluster in range (K):
                temp_sum = math.fsum(cluster_prob[:,cluster])
                new_mean = 1./ Nk[cluster]* np.sum(cluster_prob[:,cluster]*self.data.T,axis=1).T
                #print(new_mean.shape)
                parameters[cluster]['mean'] = new_mean
                diff = self.data - parameters[cluster]['mean']
                new_cov = np.array(1./ Nk[cluster]*np.dot(np.multiply(diff.T,cluster_prob[:,cluster]),diff)) 
                parameters[cluster]['cov'] = new_cov
                mix_c[cluster] = 1./ n_feat * Nk[cluster]
                
        #log likelihood
            if len(log_likelihoods)<2: continue
            if np.abs(log_likelihood-log_likelihoods[-2])<bound : break

        return mix_c, parameters

def plot_data_hist(data, name):
    _ = plt.hist(data, bins = 'auto')
    plt.title("Histogram with 'auto' bins for "+name+ " channel")
    plt.show()  

#train_data = getData("./mask_100")
train_data_1 = np.load("yellow_window_data_2.npy")
print(train_data_1.shape)
#train_data2 = np.load("yellow_window_data_25.npy")
train_data3 = np.load("yellow_window_data_50.npy")
train_data4 = np.load("yellow_window_data_75.npy")
train_data5 = np.load("yellow_window_data_100.npy")
train_data = np.concatenate([train_data_1,train_data3,train_data4,train_data5],axis=0)
print(train_data.shape)

plot_data_hist(train_data[:,0], "red")
plot_data_hist(train_data[:,1], "green")
plot_data_hist(train_data[:,2], "blue")
# input('a')

gmm = GMM(train_data)
mix_c, parameters = gmm.GMM(7)
print mix_c, parameters
save_models(mix_c, './training_params/window_weights_day_7', parameters, './training_params/gaussian_params_day_7')

