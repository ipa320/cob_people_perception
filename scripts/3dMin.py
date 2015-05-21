from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

# Convert polar (radii, angles) coords to cartesian (x, y) coords
# (0, 0) is added here. There are no duplicate points in the (x, y) plane

def multivariateGaussian(sigma,mu,value):
    return np.exp(-0.5*np.transpose(value-mu)*np.inv(sigma)*)

gridsize = 200

x,y = np.meshgrid(np.linspace(-1.0, 1.0, gridsize), np.linspace(-1.0, 1.0, gridsize))

# Pringle surface

sigma = np.array([1, 0],[0, 1])


z = np.exp(-x*y)




fig, ax = plt.subplots()

p = ax.pcolor(x, y, z, cmap=cm.RdBu, vmin=abs(z).min(), vmax=abs(z).max())
cb = fig.colorbar(p, ax=ax)

plt.show()