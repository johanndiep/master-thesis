# Johann Diep (jdiep@student.ethz.ch) - August 2019
#
# This script runs the hyperparameter optimization using the GPy Gaussian
# Process framework on Python. 

import GPy
import numpy as np
import math
import scipy.io
np.random.seed(101)

Dataset = scipy.io.loadmat('Dataset.mat')
X = Dataset['X']
Y = Dataset['Y']

k = GPy.kern.StdPeriodic(1,period=2*math.pi)      

N = 20
Z = np.random.rand(N,1)

Model = GPy.models.SparseGPRegression(X,Y,kernel=k,Z=Z)
Model.Z.constrain_bounded(-math.pi,math.pi)
Model.optimize('bfgs')
Model.plot()
print(Model)

Xi = []
for i in range(0,N):
    Xi.append(Model.inducing_inputs[i,0])

NoiseStd = math.sqrt(Model.Gaussian_noise.variance[0])
    
s0 = Model.std_periodic.variance[0]
s1 = Model.std_periodic.lengthscale[0]

Data = {'Xi':Xi,'NoiseStd':NoiseStd,'s0':s0,'s1':s1}
scipy.io.savemat('Hyperparameters.mat',Data)