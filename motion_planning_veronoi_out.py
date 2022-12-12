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
		waypoints = [[int(p[0]+north_offset), int(p[1]+east_offset), 6, 0] for p in pruned_path]
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
	
	filename = 'colliders.csv'
	data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
	drone_altitude = 6
	safety_distance = 5
	grid, edges, north_offset, east_offset = create_grid_and_edges(data, drone_altitude, safety_distance)
	start = (-north_offset, -east_offset)
	goal = (-north_offset + 256, -east_offset - 172)



	g = nx.Graph()

	for e in edges:
		p1 = e[0]
		p2 = e[1]
		dist = np.linalg.norm(np.array(p2) - np.array(p1))
		g.add_edge(p1, p2, weight=dist)

	graph_start = closest_point(g, start)
	graph_goal = closest_point(g, goal)
	path, _ = a_star_graph(g, heuristic, graph_start, graph_goal)

	print(len(path))


	plt.imshow(grid, origin='lower', cmap='Greys') 

	for e in edges:
	    p1 = e[0]
	    p2 = e[1]
	    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

	for i in range(len(path)-1):
		p1 = path[i]
		p2 = path[i+1]
		plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')

	    
	plt.plot(start[1], start[0], 'rx')
	plt.plot(goal[1], goal[0], 'rx')

	plt.xlabel('EAST')
	plt.ylabel('NORTH')
	plt.show()

	

	conn = MavlinkConnection('tcp:127.0.0.1:5760', timeout=60)

	drone = MotionDrone(conn, path, north_offset, east_offset)
	time.sleep(1)
	drone.start()