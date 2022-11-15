import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def create_voxmap(data, voxel_size=5):

	north_min = np.floor(np.min(data[:,0] - data[:,3]))
	north_max = np.ceil(np.max(data[:,0] + data[:,3]))

	east_min = np.floor(np.min(data[:,1] - data[:,4]))
	east_max = np.ceil(np.max(data[:,1] + data[:,4]))

	alt_max = np.ceil(np.max(data[:,2] + data[:,5]))

	north_size = int(np.ceil(north_max - north_min)) // voxel_size
	east_size = int(np.ceil(east_max - east_min)) //  voxel_size
	alt_size = int(alt_max) // voxel_size

	voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

	for i in range(data.shape[0]):
		north, east, alt, d_north, d_east, d_alt = data[i,:]

		obstl = [
			int(north - d_north - north_min) // voxel_size,
			int(north + d_north - north_min) // voxel_size,
			int(east - d_east - east_min) // voxel_size,
			int(east + d_east - east_min) // voxel_size
		]
		height = int(alt + d_alt) // voxel_size
		voxmap[obstl[0]:obstl[1], obstl[2]:obstl[3], 0:height] = True

	return voxmap 

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

voxmap = create_voxmap(data, 10)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxmap, edgecolor='k')
ax.set_xlim(voxmap.shape[0], 0)
ax.set_ylim(0, voxmap.shape[1])
ax.set_zlim(0, voxmap.shape[2]+70)
plt.xlabel('north')
plt.ylabel('east')
plt.show()
