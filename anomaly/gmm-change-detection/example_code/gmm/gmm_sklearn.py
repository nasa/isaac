# This code was taken from a tutorial from the following website:
# https://python-course.eu/machine-learning/expectation-maximization-and-gaussian-mixture-models-gmm.php

import matplotlib.pyplot as plt
from matplotlib import style
style.use('fivethirtyeight')
from sklearn.datasets.samples_generator import make_blobs
import numpy as np
from scipy.stats import multivariate_normal
from sklearn.mixture import GaussianMixture
from mpl_toolkits import mplot3d
from visualization import visualize_3d_gmm 

# 0. Create dataset
X,Y = make_blobs(cluster_std=0.5,random_state=20,n_features=3,n_samples=100,centers=5)

# Stratch dataset to get ellipsoid data
X = np.dot(X,np.random.RandomState(0).randn(3,3))

x,y,z = np.meshgrid(np.sort(X[:,0]),np.sort(X[:,1]),np.sort(X[:,2]))
XY = np.array([x.flatten(),y.flatten(),z.flatten()]).T

GMM = GaussianMixture(n_components=10).fit(X) # Instantiate and fit the model
print('Converged:',GMM.converged_) # Check if the model has converged
means = GMM.means_ 
covariances = GMM.covariances_


# Predict
Y = np.array([[0.5],[0.5],[0.5]])
prediction = GMM.predict_proba(Y.T)
print(prediction)

# Plot   
#fig = plt.figure(figsize=(10,10))
#ax0 = plt.axes(projection="3d")
#ax0.scatter3D(X[:,0],X[:,1],X[:,2])
#ax0.scatter3D(Y[0,:],Y[1,:],Y[2,:],c='orange',zorder=10,s=100)
for m,c in zip(means,covariances):
    multi_normal = multivariate_normal(mean=m,cov=c)
    #ax0.contour3D(np.sort(X[:,0]),np.sort(X[:,1]),np.sort(X[:,2]),multi_normal.pdf(XY).reshape(len(X),len(X),len(X)),colors='black',alpha=0.3)
    #ax0.scatter3D(m[0],m[1],m[3],c='grey',zorder=10,s=100)
visualize_3d_gmm(X, GMM.weights_,means.T,np.sqrt(covariances).T)
#plt.show()                
