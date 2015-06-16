# -*- coding: utf-8 -*-
"""
Created on Thu May  7 18:59:17 2015

@author: frm-ag
"""

import numpy as np
import matplotlib.pyplot as plt
from pylab import *



plt.close('all')

mean = [0.1,0]
cov = [[0.5,0],[0,5]] # diagonal covariance, points lie on x or y-axis


leg = np.array([-1.0,0.0])
plt.plot(leg[0],leg[1],'r.',markersize=50)

x_sample,y_sample = np.random.multivariate_normal(mean,cov,100).T

x_sample_t = []
y_sample_t = []

factor = 0.3

def transf_x(x):
    return x*np.exp(-((x-leg[0])**2))+x


for i in range(0,len(x_sample)):
    x_sample_t.append(transf_x(x_sample))
    y_sample_t.append(y_sample + (y_sample - leg[1])*factor*0.25)

# generate grid
x=linspace(-5, 5, 16)
y=linspace(-5, 5, 16)

x, y=np.meshgrid(x, y)
# calculate vector field
vx=(x-leg[0])
vy=(y-leg[1])
# plot vecor field
#quiver(x, y, vx, vy, pivot='middle', headwidth=4, headlength=6)

# plot samples
plt.plot(x_sample,y_sample,'x');
plt.plot(x_sample_t,y_sample_t,'rx'); 

plt.axis('equal'); plt.show()