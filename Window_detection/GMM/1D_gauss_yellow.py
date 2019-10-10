import numpy as np
import cv2
import copy
import sys
import matplotlib.pyplot as plt
import math
import os
import imutils
from imutils import contours

def generateData(training_data):
    stack = []
    for filename in os.listdir(training_data):
        image = cv2.imread(os.path.join(training_data,filename))
        resized = cv2.resize(image,(40,40),interpolation=cv2.INTER_LINEAR)
        image = resized[13:27,13:27]
        #cv2.imshow("imag",image)
        image_1 = image[:,:,1].ravel()
        image_2 = image[:,:,2].ravel()
        
        image = np.concatenate((image_1,image_2),axis=0)
        image = np.reshape(image,(image.shape[0],1))
        
        ch = 1
        
        # print(image.shape)
        for i in range(image.shape[0]):
            stack.append(image[i,:])
        
    return np.array(stack)

def gaussian(x,mean,cov):
        n_feat = x.shape[0] 
        
        cov_inv = 1/cov[0]
        
        diff = x-mean
       
        N = (2.0 * np.pi) ** (-len(data[1]) / 2.0) * (1.0 /cov[0] ** 0.5) *\
            np.exp(-0.5 * np.sum(np.multiply(diff*cov_inv,diff),axis=1))
        N = np.reshape(N,(n_feat,1))
        return N


def GMM(data,K):
    
    n_feat = data.shape[0] 
    n_obs = data.shape[1] 
    #n_obs = 1
   
    
    def gaussian(x,mean,cov):
        
        
        cov_inv = 1/cov[0]
        
        diff = x-mean
       
        N = (2.0 * np.pi) ** (-len(data[1]) / 2.0) * (1.0 /cov[0] ** 0.5) *\
            np.exp(-0.5 * np.sum(np.multiply(diff*cov_inv,diff),axis=1))
        N = np.reshape(N,(n_feat,1))
        return N
    

    
    def initialize():

        mean = np.array([data[np.random.choice(n_feat)]],np.float64)
        range_m = []
        
        cov = [np.random.randint(1,255)]

        return {'mean': mean, 'cov': cov}
   
   
   #generate mean and cov matrix- done
    bound = 0.0001
    max_itr = 1000
    
    parameters = [initialize() for cluster in range (K)]
    cluster_prob = np.ndarray([n_feat,K],np.float64)
    
    #EM - step E
    itr = 0
    mix_c = [1./K]*K
    log_likelihoods = []
    while (itr < max_itr):
        # print(itr)
        itr+=1
        for cluster in range (K):
            cluster_prob[:,cluster:cluster+1] = gaussian(data,parameters[cluster]['mean'],parameters[cluster]['cov'])*mix_c[cluster]
            
        
        cluster_sum = np.sum(cluster_prob,axis=1)
        log_likelihood = np.sum(np.log(cluster_sum))
        
        log_likelihoods.append(log_likelihood)
        #print(log_likelihoods)
        cluster_prob = np.divide(cluster_prob,np.tile(cluster_sum,(K,1)).transpose())
        Nk = np.sum(cluster_prob,axis = 0) #2
        #EM - step M
        for cluster in range (K):
            temp_sum = math.fsum(cluster_prob[:,cluster])
            new_mean = 1./ Nk[cluster]* np.sum(cluster_prob[:,cluster]*data.T,axis=1).T
            #print(new_mean.shape)
            parameters[cluster]['mean'] = new_mean
            diff = data - parameters[cluster]['mean']
            new_cov = np.array(1./ Nk[cluster]*np.dot(np.multiply(diff.T,cluster_prob[:,cluster]),diff)) 
            parameters[cluster]['cov'] = new_cov
            mix_c[cluster] = 1./ n_feat * Nk[cluster]
            
        if len(log_likelihoods)<2: continue
        if np.abs(log_likelihood-log_likelihoods[-2])<bound : break    
    
    return mix_c,parameters
       
    
data = generateData("data/yellow_train")
#train_data = getData()
K = 2
mix_c,parameters = GMM(data,K) 
np.save('weights_1d.npy',mix_c)
np.save('parameters_1d.npy',parameters)


name = "detectbuoy.avi"
video = []
cap = cv2.VideoCapture(name)
while (cap.isOpened()):
    success, frame = cap.read()
    if success == False:
        break    
#for filename in os.listdir("green_frames"):
    test_image = frame
    
    K = 2
    nx = test_image.shape[0]
    ny = test_image.shape[1]
   
    image_1 = test_image[:,:,1].ravel()
    image_2 = test_image[:,:,2].ravel()
        
    image = np.concatenate((image_1,image_2),axis=0)
    image = np.reshape(image,(image.shape[0],1))
    ch = 2
    
    weights = np.load('weights_1d.npy')
    parameters = np.load('parameters_1d.npy')
    
    prob = np.zeros((image.shape[0],K))
    
    likelihood = np.zeros((image.shape[0],K))
    
    for cluster in range(K):
       prob[:,cluster:cluster+1] = weights[cluster]*gaussian(image,parameters[cluster]['mean'], parameters[cluster]['cov'])
       
       likelihood = prob.sum(1)
       
   
    temp_g = likelihood[:nx*ny]
    temp_r = likelihood[nx*ny:]    
    
    green_prob = temp_g
    red_prob = temp_r
    
    prob = np.add(green_prob,red_prob)
    
    prob[red_prob>np.max(red_prob)/3.0] = 255
    output = np.zeros_like(frame)
    
    output[:,:,2]= np.reshape(prob,(nx,ny))
    output[:,:,1]= np.reshape(prob,(nx,ny))
    
    # cv2.imshow("out",output)
    blur = cv2.medianBlur(output,3)
    #cv2.imshow("blur",blur)
    edged = cv2.Canny(blur,50,255)
    # cv2.imshow("edge",edged)
    
    cnts,h = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    cnts_sorted = sorted(cnts, key=cv2.contourArea, reverse=True)
    
    hull = cv2.convexHull(cnts_sorted[0])
    (x,y),radius = cv2.minEnclosingCircle(hull)
    
    if radius > 7:
    
        cv2.circle(test_image,(int(x),int(y)-1),int(radius+1),0,4)

    cv2.imshow("Final output - Yellow buoy",test_image)
        
    
    video.append(test_image)
    cv2.waitKey(1)
    
    
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('1d_yellow.avi', fourcc, 5.0, (640, 480))
for v in video:
    out.write(v)
    cv2.waitKey(1)


out.release()
    
cap.release()
    
    
