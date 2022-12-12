from udacidrone import Drone 
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from enum import Enum, auto
import numpy as np
import time
from planning_utils import *

import msgpack
from udacidrone.frame_utils import global_to_local
from skimage.morphology import medial_axis
from skimage.util import invert


class Phases(Enum):
	MANUEL = auto()
	ARM = auto()
	DISARM = auto()
	TAKEOFF = auto()
	LAND = auto()
	WAYPOINT = auto()
	PLANNING = auto()


class Motion_Planning(Drone):
	def __init__(self, connection):
		super().__init__(connection)

		self.flight_phase = Phases.MANUEL
		self.in_mission = True 
		self.target_position = np.array([0.0, 0.0, 0.0])
		self.all_waypoints = []


		self.register_callback(MsgID.STATE, self.state_callback)
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.local_velocity_callback)




	def local_position_callback(self):
		if self.flight_phase == Phases.TAKEOFF:
			altitude = -1.0 * self.local_position[2]

			if altitude > 0.95 * self.target_position[2]:
				self.waypoint_transition()
		elif self.flight_phase == Phases.WAYPOINT:
			if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
				if len(self.all_waypoints) > 0:
					self.waypoint_transition()
				else:
					if np.linalg.norm(self.local_velocity[0:2] < 1.0):
						self.land_transition()


	def local_velocity_callback(self):
		if self.flight_phase == Phases.LAND:
			if self.global_position[2] - self.global_home[2] < 0.5 :
				if abs(self.local_position[2]) < 0.01:
					self.disarm_transition()



	def state_callback(self):
		if self.in_mission:
			if self.flight_phase == Phases.MANUEL:
				self.arming_transition()
			elif self.flight_phase == Phases.ARM:
				if self.armed:
					self.planning_transition()
			elif self.flight_phase == Phases.PLANNING:
				self.takeoff_transition()
			elif self.flight_phase == Phases.DISARM:
				if ~self.armed & ~self.guided:
					self.manuel_transition()

	

	def calculate_box(self):
		print("setting home")
		local_waypoints = [[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]
		return local_waypoints

	def waypoint_transition(self):
		self.flight_phase = Phases.WAYPOINT
		print('waypoint_transition')
		self.target_position = self.all_waypoints.pop(0)
		print('target position ', self.target_position)
		self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)

	def visual_waypoints(self):
		print("sending waypoints to simulateur...")
		data = msgpack.dumps(self.all_waypoints)
		self.connection._master.write(data)

	def planning_transition(self):
		print('planning transition')
		self.flight_phase = Phases.PLANNING
		filename = 'colliders.csv'
		data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

		#retrieve drone geodic coordinates
		lat = self._latitude
		lon = self._longitude
		alt = self._altitude

		#set the start position to the currenct position
		start_ned = global_to_local((lon, lat, alt), self.global_home)
		goal_ned = global_to_local((-122.397486,37.792845, 0), self.global_home)

		
		
		safety_distance = 5
		drone_altitude = 5
		self.target_position[2] = drone_altitude

		grid, north_offset, east_offset = create_grid(data, drone_altitude, safety_distance)
		grid_start = (int(start_ned[0] - north_offset), int(start_ned[1] - east_offset))
		grid_goal = (int(goal_ned[0]) - north_offset, int(goal_ned[1]) - east_offset)

		skeleton = medial_axis(invert(grid))
		skel_start, skel_goal = medial_axis_find_start_goal(skeleton, start, goal)

		path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(skel_start), tuple(skel_goal))
		final_path = prune_path(path)
		waypoints = [[p[0] + north_offset, p[1] + east_offset, drone_altitude, 0] for p in final_path]
		self.all_waypoints = waypoints
		self.visual_waypoints()

	












	def arming_transition(self):
		print('arming transition')
		#set global home from the first line of the 'colliders.csv'
		with open('colliders.csv', 'r') as file:
			first_line = file.readline()
			self.set_home_position(float(first_line.split(" ")[3]), float(first_line.split(" ")[1].split(",")[0]), 0.0)
		
		self.arm()
		self.take_control()
		self.flight_phase = Phases.ARM 

	def disarm_transition(self):
		print('disarm transition')
		self.disarm()
		self.flight_phase = Phases.DISARM

	def takeoff_transition(self):
		print('takeoff transition')
		self.takeoff(self.target_position[2])
		self.flight_phase = Phases.TAKEOFF 

	def land_transition(self):
		print('landing transition')
		self.land()
		self.flight_phase = Phases.LAND 
	
	def manuel_transition(self):
		print('manuel transition')
		self.release_control()
		self.stop()
		self.in_mission = True 
		self.flight_phase = Phases.MANUEL 
	def start(self):
		self.start_log('Logs', 'NavLog.txt')
		print("starting connection")
		self.connection.start()
		self.stop_log()


	


if __name__ == '__main__':
	conn = MavlinkConnection('tcp:127.0.0.1:5760', timeout=300)
	drone = Motion_Planning(conn)
	time.sleep(2)
	drone.start()

	












