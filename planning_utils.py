from enum import Enum
from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham
from shapely.geometry import Polygon, LineString, Point
from sklearn.neighbors import KDTree
import networkx as nx


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def create_grid_and_edges(data, drone_altitude, safety_distance):

    north_min = np.floor(np.min(data[:,0] - data[:,3]))
    north_max = np.ceil(np.max(data[:,0] + data[:,3]))

    east_min = np.floor(np.min(data[:,1] - data[:,4]))
    east_max = np.ceil(np.max(data[:,1] + data[:,4]))

    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    grid = np.zeros((north_size, east_size))
    points = []

    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size -1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1]=1

            points.append([north - north_min, east - east_min])

    graph = Voronoi(points)
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]

        cells = list(bresenham(int(p1[0]),int(p1[1]), int(p2[0]),int(p2[1])))
        hit = False

        for c in cells:
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1]>=grid.shape[1]:
                hit = True
                break

            if grid[c[0], c[1]] == 1:
                hit = True
                break

        if not hit:
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, int(north_min), int(east_min)






# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, 1.4)
    NORTH_EAST = (-1, 1, 1.4)
    SOUTH_WEST = (1, -1, 1.4)
    SOUTH_EAST = (1, 1, 1.4)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    off_north = x - 1 < 0
    off_south = x + 1 > n
    off_west = y - 1 < 0
    off_east = y + 1 > m 

    if off_north or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if off_south or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if off_west or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if off_east or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if off_north or off_east or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if off_north or off_west or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if off_south or off_east or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if off_south or off_west or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)



    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def a_star_graph(graph, heuristic, start, goal):
    path = []
    path_cost = 0
    visited = set(start)
    q = PriorityQueue()
    q.put((0, start))
    branch = {}
    found = False

    while not q.empty():
        item = q.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][1]
        if current_node == goal:
            print('path found')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = cost + current_cost
                q_cost = branch_cost + heuristic(next_node, goal)
                if next_node not in visited:
                    visited.add(next_node)
                    q.put((q_cost, next_node))
                    branch[next_node] = (current_node, branch_cost)

    if found:
        n = goal
        path_cost = branch[n][1]
        path.append(n)
        while branch[n][0] != start:
            path.append(branch[n][0])
            n = branch[n][0]

        path.append(branch[n][0])
    else:
        print("failed to find a path")

    return path[::-1], path_cost


def medial_axis_find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]

    return near_start, near_goal








def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def closest_point(graph, current_point):
    closest_point = None 
    dist = 10000
    for p in graph.nodes:
        d = np.linalg.norm(np.array([p[0],p[1]])-np.array(current_point))
        if d < dist:
            closest_point = p 
            dist = d 
    return closest_point


def point(p):
        return np.array([p[0], p[1], 1.0]).reshape(1,-1)

def check_collinearity(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [x for x in path]
    i = 0
    while i < len(pruned_path)-2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])

        if not check_collinearity(p1, p2, p3):
            i += 1
            continue

        pruned_path.remove(pruned_path[i+1])
    return pruned_path

def can_connect(polygons, n1, n2):
    line = LineString([n1, n2])
    for p in polygons:
        if p.crosses(line) and p.height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(polygons, nodes, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        idxs = tree.query([n1], k, return_distance=False)[0]

        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue

            if can_connect(polygons, n1, n2):
                g.add_edge(n1, n2, weight=1)
    return g






