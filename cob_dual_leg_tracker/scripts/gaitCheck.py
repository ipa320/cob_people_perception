# -*- coding: utf-8 -*-
"""
Created on Wed May  6 13:53:54 2015

@author: frm-ag
"""
import matplotlib.pyplot as plt
import numpy as np
import scipy.fftpack
from savitzky_golay import savitzky_golay

left_leg = [0.183579,0.491983,0.0154937,0.0212442,0.0214935,0.00250426,0.00890727,0.00546054,0.022119,0.049585,0.0788872,0.168854,0.181998,0.179334,0.16477,0.0953063,0.0770077,0.049496,0.0238781,0.015671,0.0102151,0.00526309,0.0095898,0.00956311,0.00940341,0.0089271,0.0129194,0.0149079,0.0272394,0.0360702,0.0767114,0.112875,0.105041,0.0673649,0.0378214,0.0279475,0.0173993,0.00888539]
right_leg = [0.034484,0.0418171,0.118991,0.187956,0.220516,0.18639,0.163862,0.073867,0.0587981,0.0294763,0.0278228,0.0112316,0.0114986,0.0049741,0.00457483,0.0112099,0.00615317,0.0333899,0.0307088,0.0731042,0.130397,0.151966,0.176118,0.169479,0.134661,0.0788245,0.0595687,0.0326763,0.0219942,0.0162864,0.0275825,0.00807434,0.0107357,0.0040571,0.00865209,0.00110538,0.00508876,0.000946234]

s = []

# Apply filters to the data
filter = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
filter = filter/sum(filter) # normalize

print filter


w = scipy.fftpack.rfft(left_leg)
f = scipy.fftpack.rfftfreq(len(left_leg), 1)
spectrum = w**2

cutoff_idx = spectrum < (spectrum.max()/5)
w2 = w.copy()
w2[cutoff_idx] = 0

y2 = scipy.fftpack.irfft(w2)


#x = np.linspace(0,2*np.pi,100)
#y = np.sin(x) + np.random.random(100) * 0.2




left_leg = np.convolve(left_leg,filter, mode='same')
right_leg = np.convolve(right_leg,filter, mode='same')

left_leg_smooth = savitzky_golay(np.array(left_leg), 11, 3) # window size 51, polynomial order 3
right_leg_smooth =savitzky_golay(np.array(right_leg), 13, 3) # window size 51, polynomial order 3

# Reverse order for test purposes
#l0 = l0[::-1]

# Calculate the norm
for i in range(0,len(left_leg)):
    s.append(left_leg[i]*right_leg[i])
    
print np.trapz(s)
    
plt.close('all')
plt.plot(left_leg,label="leg0")
plt.plot(right_leg,label="leg1")
plt.plot(s,label="product")

plt.plot(left_leg_smooth)
plt.plot(right_leg_smooth)


print  sum(s)/len(s)

plt.legend()
plt.xlabel("time[steps]")
plt.ylabel("distance to previous position[m]")

