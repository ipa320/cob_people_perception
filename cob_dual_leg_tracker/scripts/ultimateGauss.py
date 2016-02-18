# -*- coding: utf-8 -*-
"""
Created on Tue May 19 13:39:39 2015

@author: frm-ag
"""

import numpy as np
import random
import matplotlib.pyplot as plt

N = 10000
samples = []

v1 = np.array([1.0,2.0])
v2 = np.array([-2.0,1.0])*0.5


for i in range(0,N):
    v1_factor = random.gauss(0,1)*3
    
    # Do a fifty fifty chance
    if(random.random() < 0.5):
        v2_factor = np.abs(random.gauss(0,v1_factor))
    else:
        v2_factor = -np.abs(random.gauss(0,1))*2
    
    samples.append(v1*v1_factor + v2*v2_factor);
    
samples = np.array(samples)

x = samples[:,0]
y = samples[:,1]

plt.plot(x,y,'x');

# Plot the vectors
plt.plot([0, v1[0]],[0, v1[1]],'r');
plt.plot([0, v2[0]],[0, v2[1]],'g');

plt.axis('equal'); plt.show()