import argparse
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
import numpy as np
from udacidrone.messaging import MsgID
from enum import Enum, auto
import time
import math
import msgpack
from planning_utils import *
from udacidrone.frame_utils import global_to_local
from skimage.morphology import medial_axis
from skimage.util import invert
import matplotlib.pyplot as plt
import networkx as nx
from sampling import Sampler



class State(Enum):
	ARMING = auto()
	DISARMING = auto()
	TAKEOFF = auto()
	LANDING = auto()
	MANUAL = auto()
	WAYPOINT = auto()
	PLANNING = auto()


class MotionDrone(Drone):

	def __init__(self, connection, path, north_offset, east_offset):
		super().__init__(connection)
		self.target_position = np.array([0.0, 0.0, 0.0])
		self.all_waypoints = []
		self.in_mission = True
		self.check_state = {}
		self.path = path
		self.north_offset = north_offset
		self.east_offset = east_offset

        # initial state
		self.flight_phase = State.MANUAL

        # register all your callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)


	def local_position_callback(self):
		if self.flight_phase == State.TAKEOFF:
			if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
				self.waypoint_transition()
		elif self.flight_phase == State.WAYPOINT:
			if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 2.0:
				if len(self.all_waypoints) > 0:
					self.waypoint_transition()
				else:
					if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
						self.landing_transition()



	def velocity_callback(self):
		if self.flight_phase == State.LANDING:
			if self.global_position[2] - self.global_home[2] < 0.1:
				if abs(self.local_position[2] < 0.01):
					self.disarming_transition()


	def send_waypoints(self):
		print("sending waypoints to simulateur...")
		data = msgpack.dumps(self.all_waypoints)
		self.connection._master.write(data)




	def state_callback(self):
		if self.in_mission:
			if self.flight_phase == State.MANUAL:
				self.arming_transition()
			elif self.flight_phase == State.ARMING:
				if self.armed:
					self.planning_transition()
			elif self.flight_phase == State.PLANNING:
				self.takeoff_transition()
			elif self.flight_phase == State.DISARMING:
				self.manual_transition()

	

	def waypoint_transition(self):
		self.flight_phase = State.WAYPOINT
		print('waypoint transition')
		self.target_position = self.all_waypoints.pop(0)
		print(self.target_position)
		self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])




	def planning_transition(self):
		self.flight_phase = State.PLANNING 
		print('planning transition')
		pruned_path =prune_path(path)
		waypoints = [[int(p[0]), int(p[1]), int(p[2]), 0] for p in path]
		self.all_waypoints = waypoints
		self.send_waypoints()

	




	def arming_transition(self):
		self.flight_phase = State.ARMING 
		print('arming transition')
		self.arm()
		self.take_control()

	def disarming_transition(self):
		self.flight_phase = State.DISARMING
		print('disarming transition')
		self.disarm()
		self.release_control()

	def landing_transition(self):
		self.flight_phase = State.LANDING
		print('landing_transition')
		self.land()

	def takeoff_transition(self):
		self.flight_phase = State.TAKEOFF
		print('takeoff transition')
		self.takeoff(self.target_position[2])

	def manual_transition(self):
		self.flight_phase = State.MANUAL
		print('manual transition')
		self.in_mission = False
		self.stop()

	def start(self):
		self.start_log("Logs", "NavLog.txt")

		print("starting connection")
		self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

		self.stop_log()

	






if __name__ == '__main__':

	#load the data
	
	filename = 'colliders.csv'
	data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

	#sample points

	sampler = Sampler(data)
	polygons = sampler._polygons
	print(len(polygons))
	nodes = sampler.sample(300)
	print(len(nodes))

	graph = create_graph(polygons, nodes, 10)
	print(len(graph.edges))
	grid, north_offset, east_offset = create_grid(data, sampler._zmax, 1)

	start = (0, 0)
	goal = (256,- 172)

	graph_start = closest_point(graph, start)
	graph_goal = closest_point(graph, goal)
	print(graph_start,graph_goal)
	print(start, goal)

	start = list(graph.nodes)[0]
	k = np.random.randint(len(graph.nodes))
	print(k, len(graph.nodes))
	goal = list(graph.nodes)[k]
	path, _ = a_star_graph(graph, heuristic, graph_start, graph_goal)
	print(len(path))

	path_pairs = zip(path[:-1], path[1:])
	for (n1, n2) in path_pairs:
		print(n1, n2)

	fig = plt.figure()

	plt.imshow(grid, cmap='Greys', origin='lower')

	nmin = np.min(data[:, 0])
	emin = np.min(data[:, 1])

	# draw nodes
	for n1 in graph.nodes:
	    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
	    
	# draw edges
	for (n1, n2) in graph.edges:
	    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')
	    
	path_pairs = zip(path[:-1], path[1:])
	for (n1, n2) in path_pairs:
	    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')


	plt.xlabel('NORTH')
	plt.ylabel('EAST')

	plt.show()





	

	conn = MavlinkConnection('tcp:127.0.0.1:5760', timeout=60)

	drone = MotionDrone(conn, path, north_offset, east_offset)
	time.sleep(1)
	drone.start()