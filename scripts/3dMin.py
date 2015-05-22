from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np
import random

plt.close('all')

def multivariateGaussian(sigma,mu,value):
    sigma_inv =     np.linalg.inv(sigma)
    diff = value-mu
    #print sigma
    #print "mu= ", mu   
    #print "value= ", value
    #print "diff= ", diff
    
    return_value = np.exp(-0.5* np.transpose(diff).dot(sigma_inv.dot(diff)))
    return return_value
    
def multivariateGaussianDeriv(sigma,mu,value):
    sigma_inv =     np.linalg.inv(sigma)
    diff = value-mu
    return -multivariateGaussian(sigma,mu,value) * sigma_inv.dot(diff)
    
def multimultivariateGaussian(sigmas_list,mus_list,value):
    total = np.array(0.0);
    for i in range(0,N_dist):
        total = total + multivariateGaussian(sigmas_list[i],mus_list[i],value)
    return total
    
def multimultivariateGaussianDeriv(sigmas_list,mus_list,value):
    total = np.zeros([2,1]);
    for i in range(0,N_dist):
        total = total + multivariateGaussianDeriv(sigmas_list[i],mus_list[i],value)
    return total

# TODO Generate Multiple Sigmas
N_dist = 15

sigmas_list = []
mus_list = []
for i in range(0,N_dist):
    # Generate sigma(cov)
    A =  np.random.random((2,2))    
    
    sigma = (A + A.T)/2
    sigma = np.array([[1,0],[0,1]])*0.01
    sigmas_list.append(sigma)
    
    #Generate mu(mean)
    mus_list.append(np.random.uniform(-0.6,0.6,[2,1]))

# Set the gaussian parameters
value = np.empty([2,1])
#mu = np.array([0, 0]);
#sigma = np.array([[1, 0],[0, 10]])

gridsize = 50

x,y = np.meshgrid(np.linspace(-1, 1, gridsize), np.linspace(-1, 1, gridsize))

z = np.zeros(shape=(gridsize,gridsize))


for i in range(0,len(x)):
    for j in range(0,len(y)):
        value[0] = x[i,j]
        value[1] = y[i,j]
        
        z[i,j] = multimultivariateGaussian(sigmas_list, mus_list, value)
       
    
# Calculate the derivative    
# x_delta = np.zeros(shape=(gridsize,gridsize))
# y_delta = np.zeros(shape=(gridsize,gridsize))    
#for i in range(0,len(x)):
#    for j in range(0,len(y)):
#        value[0] = x[i,j]
#        value[1] = y[i,j]
#        deriv = multimultivariateGaussianDeriv(sigmas_list, mus_list, value)
#        x_delta[i,j] = deriv[0]
#        y_delta[i,j] = deriv[1]
        
# Do newton
min_est = []        
start_point = np.array([0.0, 0.0]).T
min_est.append(start_point)

steps = 2000
step_size = 0.05
        
for i in range(0,steps):
    last_value = np.reshape(min_est[i],[2,1])
    
    new_point = last_value-multimultivariateGaussianDeriv(sigmas_list,mus_list,last_value)*step_size
    min_est.append(new_point)

#z = np.exp(-x*y)



# Plot the costs
fig = plt.figure()
ax = fig.add_subplot(211)


p = ax.pcolor(x, y, z, cmap=cm.RdBu, vmin=z.min(), vmax=z.max())
cb = fig.colorbar(p, ax=ax)


#p = ax.plot_surface(x, y, z,  rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=True)

#ax = fig.add_subplot(212)
#Q = plt.quiver(x,y, x_delta, y_delta)
plt.plot(np.array(min_est)[:,0],np.array(min_est)[:,1])
ax.text(min_est[0][0], min_est[0][1],'Start', fontsize=8)
ax.text(min_est[-1][0], min_est[-1][1],'End', fontsize=8)

plt.show()