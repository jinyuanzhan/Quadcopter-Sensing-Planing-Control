from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

import heapq
from heapq import heappush, heappop

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    # Return a tuple (path, nodes_expanded)
    pq = [] # maintain a priority queue to track the cost to come
    parents = {} # Use a dictionary to track the parent node of each node
    cost_to_come = {} # Use a dictionary to track the cost to come for each node
    visited = set()
    goal_reached = False
    nodes_expanded = 0
    cost_to_come[start_index] = 0

    # Initialize:
    # Find all the nodes in the map, assign them with infinite cost and put them in the pq if they are not occupied:
    # map_X, map_Y, map_Z = occ_map.map.shape
    
    # for i in range(map_X):
    #     for j in range(map_Y):
    #         for k in range(map_Z):
    #             node_index = (i, j, k)
    #             if occ_map.is_valid_index(node_index) and not occ_map.is_occupied_index(node_index):
    #                 heappush(pq, (np.inf, node_index))

    start_heuristic = np.linalg.norm((np.array(start_index) - np.array(goal_index)) * occ_map.resolution) if astar else 0
    f_start = cost_to_come[start_index] + start_heuristic
    heappush(pq, (f_start, start_index))


    # when we haven't reached the goal and there are still nodes to be expanded in the pq
    while not goal_reached and pq:
        current_cost, current_index = heappop(pq)
        if current_index in visited:
            continue
        visited.add(current_index)
        nodes_expanded += 1
        if current_index == goal_index:
            goal_reached = True
            path = []
            temp_node = goal_index
            while temp_node != start_index:
                path.append(occ_map.index_to_metric_center(temp_node))
                temp_node = parents[temp_node]
            path.append(occ_map.index_to_metric_center(start_index))
            path.reverse()
            # Note that we were using index_to_metric_center to convert index to metrix number, but the start and goal are fixed and not necessarily be the center, so they should be overwritten to the true value.
            path[0] = start
            path[-1] = goal
            path = np.array(path)
            return path, nodes_expanded

        # if current_cost > cost_to_come.get(current_index, np.inf):
        #     continue
        current_neighbours = index_neighbours(current_index)
        for neighbour in current_neighbours:
            if neighbour in visited:
                continue
            if occ_map.is_valid_index(neighbour) and not occ_map.is_occupied_index(neighbour):
                edge_cost = np.linalg.norm((np.array(current_index) - np.array(neighbour)) * occ_map.resolution)
                g_cost = cost_to_come[current_index] + edge_cost
                heuristic = np.linalg.norm((np.array(neighbour) - np.array(goal_index)) * occ_map.resolution) if astar else 0
                f_cost = g_cost +1.7 * heuristic
                # cost.get(neighbor, np.inf) If neighbour exists in dict, then return its value, otherwise inf
                if g_cost < cost_to_come.get(neighbour, np.inf):
                    cost_to_come[neighbour] = g_cost
                    parents[neighbour] = current_index
                    heappush(pq, (f_cost, neighbour))
            else: 
                continue
    return None, nodes_expanded

NEIGHBOR_OFFSETS = [(dx, dy, dz)
                    for dx in [-1, 0, 1]
                    for dy in [-1, 0, 1]
                    for dz in [-1, 0, 1]
                    if not (dx == 0 and dy == 0 and dz == 0)]

def index_neighbours(current_index):
    # index_x, index_y, index_z = current_index
    # neighbours = []
    # neighbour_x_range = [index_x-1, index_x, index_x+1]
    # neighbour_y_range = [index_y-1, index_y, index_y+1]
    # neighbour_z_range = [index_z-1, index_z, index_z+1]
    # for i in neighbour_x_range:
    #     for j in neighbour_y_range:
    #         for k in neighbour_z_range:
    #             if i == index_x and j == index_y and k == index_z:
    #                 continue
    #             else:
    #                 neighbours.append((i, j, k))
    cx, cy, cz = current_index
    neighbours = [(cx + dx, cy + dy, cz + dz) for dx, dy, dz in NEIGHBOR_OFFSETS]
    return neighbours






