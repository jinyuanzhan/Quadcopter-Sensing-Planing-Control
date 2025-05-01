
from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World

from ..util.occupancy_map import OccupancyMap # Recommended.

def graph_search(occ_map, resolution, margin, start, goal,goal_tolerance ,astar):
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
    #----------------------------
    # function1 to get center of the cell.
    center = {}
    def get_center(idx):
        if idx not in center:
            center[idx] = occ_map.index_to_metric_center(idx)
        return center[idx]
    #----------------------------
    
    #----------------------------
    # function2 weighted A* heuristic.
    weight = 1
    def heuristic(index):
        if not astar:
            return 0
        # return np.linalg.norm(get_center(idx) - goal_center)
        # L2 norm euclidean distance
        return weight * np.linalg.norm(get_center(index) - goal_center)
    #----------------------------
    # occ_map = OccupancyMap(world, resolution, margin)
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index  = tuple(occ_map.metric_to_index(goal)) 
    print("start_index: ", start_index)
    if occ_map.is_occupied_index(start_index) or occ_map.is_occupied_index(goal_index):
        return None, 0

    #------------------------------
    frontier = []                          # contain (priority, index)
    came_from_dict = {start_index: None}   # recode came from
    cost_so_far_dict = {start_index: 0}    # recode cost
    visited = set()                        # recode visited
    nodes_expanded = 0
    #------------------------------
    # 3 x 3 x 3  - self = 26
    neighbor_offsets = []
    for x in (1, 0, -1):
        for y in (1, 0, -1):
            for z in (1, 0, -1):
                #self
                if (x, y, z) != (0, 0, 0):
                    neighbor_offsets.append((x, y, z)) # 26 neighbors
    #------------------------------

    goal_center = get_center(goal_index)
    # format heappush: (heap, (priority, index))
    heappush(frontier, (cost_so_far_dict[start_index] + heuristic(start_index), start_index))

    neighbor_costs = {}
    rx, ry, rz = resolution
    for offset in neighbor_offsets:
        x, y, z = offset
        neighbor_costs[offset] = np.sqrt((x*rx)**2 + (y * ry)**2 + (z * rz)**2)
    # ---------------------------


    while frontier:
        a, current = heappop(frontier)
        if current in visited:
            continue
        nodes_expanded += 1
        visited.add(current)

        # if current == goal_index:
        #     break
        # ----------------
        
        goal_distance = np.linalg.norm(get_center(current) - goal_center)
        if goal_distance <= goal_tolerance and not occ_map.is_occupied_index(current):
           
            goal_index = current 
            break
        # ----------------

        for offset in neighbor_offsets:
            # to check 
            neighbor = tuple(map(sum, zip(current, offset)))
            if not occ_map.is_valid_index(neighbor) or occ_map.is_occupied_index(neighbor):
                continue
            new_cost = cost_so_far_dict[current] + neighbor_costs[offset]
            if neighbor not in cost_so_far_dict or (new_cost < cost_so_far_dict[neighbor]):
                cost_so_far_dict[neighbor] = new_cost # asign new cost to a key called neighbor
                came_from_dict[neighbor] = current
                priority = new_cost + heuristic(neighbor) # f(n) = g(n) + h(n)  cost
                heappush(frontier, (priority, neighbor))
    # ---------------------------

    # if goal_index not in came_from_dict and goal_index != start_index:
    if goal_index not in came_from_dict:
        # No path found cases path []
        return None, nodes_expanded
    
    path_index = []
    cur = goal_index # tuple(x, y, z)
    while cur is not None:
        path_index.append(cur)
        cur = came_from_dict[cur] # came_from_dict[cur] = previous node 

    path_index.reverse()

    # convert to cell centers to real coordinates
    path = np.empty((len(path_index), 3))  
    for i, index in enumerate(path_index):
        path[i] = get_center(index)
    path[0]  = start
    path[-1] = goal
    # print("path: ", path)
    return path, nodes_expanded