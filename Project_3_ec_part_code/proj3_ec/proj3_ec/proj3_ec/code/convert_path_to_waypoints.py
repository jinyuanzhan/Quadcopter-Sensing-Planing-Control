# import numpy as np

# def convert_path_to_waypoints(path, resolution, margin):
#     """
#     Convert a path to a list of waypoints.

#     Inputs:
#         path, path from graph search shape =(N,3)
#         resolution, xyz resolution in meters for an occupancy map, shape=(3,)
#         margin, minimum allowed distance in meters from path to obstacles.
#     Outputs:
#         waypoints, list of selected waypoints
#     """
#     num_points = path.shape[0] # N
#     waypoints = list()
#     waypoints.append(path[0])

#     for i in range (1,num_points-1):
#         v1 = path[i-1] - path[i]
#         v2 = path[i+1] - path[i]
#         v1_norm = np.linalg.norm(v1)
#         v2_norm = np.linalg.norm(v2)
#         if v1_norm == 0 or v2_norm == 0:
#             continue
#         angle_between = np.arccos(np.dot(v1,v2)/(v1_norm*v2_norm))
#         if (angle_between) >= np.pi/4:
#             waypoints.append(path[i])
#     waypoints.append(path[-1])
#     waypoints = np.array(waypoints)
#     return waypoints
        
        
    
# import numpy as np

# def point_line_distance(point, start, end):
#     if np.allclose(start, end):
#         return np.linalg.norm(point - start)
#     return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

# def douglas_peucker(points, epsilon):
#     if points.shape[0] < 3:
#         return points
#     max_distance = 0.0
#     index = 0
#     for i in range(1, len(points) - 1):
#         d = point_line_distance(points[i], points[0], points[-1])
#         if d > max_distance:
#             index = i
#             max_distance = d
#     if max_distance > epsilon:
#         rec_results1 = douglas_peucker(points[:index+1], epsilon)
#         rec_results2 = douglas_peucker(points[index:], epsilon)
#         result = np.vstack((rec_results1[:-1], rec_results2))
#     else:
#         result = np.array([points[0], points[-1]])
    
#     return result
# def convert_path_to_waypoints(path, resolution, margin):
#     epsilon = 0.1
#     # epsilon 越低, 结果越接近原始路径
#     waypoints = douglas_peucker(path, epsilon)
#     return waypoints

import numpy as np
def point_line_distance(point, start, end):
    if np.allclose(start, end):
        return np.linalg.norm(point - start)
    return np.linalg.norm(np.cross(end - start, start - point)) / np.linalg.norm(end - start)

def douglas_peucker(points, epsilon):
    if points.shape[0] < 3:
        return points
    max_distance = 0.0
    index = 0
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], points[0], points[-1])
        if d > max_distance:
            index = i
            max_distance = d
    if max_distance > epsilon:
        rec_results1 = douglas_peucker(points[:index+1], epsilon)
        rec_results2 = douglas_peucker(points[index:], epsilon)
        result = np.vstack((rec_results1[:-1], rec_results2))
    else:
        result = np.array([points[0], points[-1]])
    return result

def add_midpoints(waypoints, max_seg_length):
    new_waypoints = [waypoints[0]]
    for i in range(1, len(waypoints)):
        prev = waypoints[i-1]
        curr = waypoints[i]
        seg_distance = np.linalg.norm(curr - prev)
        if seg_distance > max_seg_length:
            n_midpoints = int(np.ceil(seg_distance / max_seg_length)) - 1
            for j in range(1, n_midpoints + 1):
                new_point = prev + (curr - prev) * (j / (n_midpoints + 1))
                new_waypoints.append(new_point)
        new_waypoints.append(curr)
    return np.array(new_waypoints)

def convert_path_to_waypoints(path, resolution, margin):
    epsilon = 0.2
    waypoints = douglas_peucker(path, epsilon)
    max_seg_length = 0.9
    waypoints_with_midpoints = add_midpoints(waypoints, max_seg_length)
    return waypoints_with_midpoints
