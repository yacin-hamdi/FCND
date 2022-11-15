import numpy as np 
import matplotlib.pyplot as plt
from grid import create_grid
from shapely.geometry import Polygon, Point



#this function used for creating polygons its takes data which is the data  
#from colliders.csv file and return polygons of the obstacles
def create_polygon(data):
	polygons = []
	for i in range(data.shape[0]):
		north, east, alt, d_north, d_east, d_alt = data[i,:]

		obscl = [north - d_north, north + d_north, east - d_east, east + d_east]
		corners = [(obscl[0], obscl[2]), (obscl[0], obscl[3]), (obscl[1], obscl[3]), (obscl[1], obscl[2])]
		polygon = Polygon(corners)
		height = alt + d_alt
		polygons.append((polygon, height))

	return polygons


#return true if point is in collusion with polygon else it return false
def collides(polygons, point):
	for (polygon, height) in polygons:
		if polygon.contains(Point(point)) and height >= point[2]:
			return True
	return False


#this function generate random 3d points
def generate_samples(data, num_samples=500):
	xmin = np.min(data[:,0] - data[:,3])
	xmax = np.max(data[:,0] + data[:,3])

	ymin = np.min(data[:,1] - data[:,4])
	ymax = np.max(data[:,1] + data[:,4])

	zmin = 0
	zmax = 10

	xvals = np.random.uniform(xmin, xmax, num_samples)
	yvals = np.random.uniform(ymin, ymax, num_samples)
	zvals = np.random.uniform(zmin, zmax, num_samples)

	samples = list(zip(xvals, yvals, zvals))

	return samples 

#return all point that not in collusion with polygons
def to_keep(polygons, samples):
	keep = []
	for point in samples:
		if not collides(polygons, point):
			keep.append(point)
	return keep

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

print('please wait..')
polygons =  create_polygon(data)
samples = generate_samples(data, 1000)
keep = np.array(to_keep(polygons, samples))

drone_altitude = 10
safety_distance = 1
grid = create_grid(data, drone_altitude, safety_distance)
print('finished')


fig = plt.figure()
plt.imshow(grid, cmap='Greys', origin='lower')
nmin = np.min(data[:,0])
emin = np.min(data[:,1])
north_vals = keep[:,0]
east_vals = keep[:,1]
plt.scatter(east_vals - emin, north_vals - nmin, c='red')
plt.xlabel('east')
plt.ylabel('north')
plt.show()
