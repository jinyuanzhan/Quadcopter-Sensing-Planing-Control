from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World


def graph_search(occ_map, start, goal, astar):
    """
    Parameters:
        occ_map,    Occ map object representing the environment obstacles
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

    # While not required, we have provided an occupancy map you can use.
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))

    
    # TODO Your code here -   copy your previous code in proj1.2 with provided "occ_map".
    # You may need to relax the exit condition to allow searching while goal is occupied.

    # Return a tuple (path, nodes_expanded)
    return None, 0
