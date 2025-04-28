import numpy as np
from scipy.interpolate import CubicSpline

from scipy.interpolate import make_interp_spline
from .graph_search import graph_search

from scipy.interpolate import make_interp_spline
import numpy as np


# t = np.array([0, 1, 2, 3, 4])
# x = np.array([0, 2, 1, 3, 2])


# spline = make_interp_spline(t, x, k=5)  #


# t_eval = np.linspace(0, 4, 100)
# x_eval = spline(t_eval)
# x_dot = spline.derivative(1)(t_eval)
# x_ddot = spline.derivative(2)(t_eval)


class WorldTraj(object):
    """
    Generates a smooth trajectory from start to goal while avoiding obstacles.
    """

    def __init__(self, world, start, goal):
        """
        Initializes the trajectory by computing a smooth path from start to goal.

        Parameters:
            world: World object representing the environment obstacles.
            start: xyz start position in meters, shape=(3,)
            goal:  xyz goal position in meters, shape=(3,)
        """
        # Set initial grid resolution & safety margin
        self.resolution = np.array([0.1, 0.1, 0.1])  # Grid resolution
        self.initial_margin = 0.4
        self.margin_step = 0.05

        ##
        # Minimum allowable margin
        self.min_margin = 0.4
        # Storing world
        self.world = world  

        # Computing initial path using A* 

        self.margin = self.initial_margin
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        ### Trying again loop

        # If no path is found, decreasing the margin
        if self.path is None or len(self.path) < 2:
            print(f"No valid path found with margin {self.margin:.2f}, decreasing margin...")
            while self.margin > self.min_margin:
                self.margin -= self.margin_step
                print(f"Trying margin: {self.margin:.2f}")
                self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
                if self.path is not None and len(self.path) >= 2:
                    print(f"Path found with margin {self.margin:.2f}")
                    break
            if self.path is None or len(self.path) < 2:
                raise ValueError("No valid path found even after adjustments!")

        # Extracting sparse waypoints from the dense path using RDP & collinearity check
        sparse_path = self._rdp_sparsify(self.path, epsilon=0.26)  
        self.points = self._remove_collinear_points(sparse_path)

        # Computing segment times using adaptive velocity 
        self.segment_times = self._compute_segment_times()



        # Generating smooth cubic spline trajectory = min Acc
        self._compute_smooth_trajectory()


    ## RDP
    def _rdp_sparsify(self, path, epsilon=0.2):
        """
        Implements the Ramer-Douglas-Peuckeralgorithm to remove redundant points while preserving curvature

        """
        def perpendicular_distance(pt, start, end):
        
            if np.allclose(start, end):
                return np.linalg.norm(pt - start)
            return np.linalg.norm(np.cross(end - start, start - pt)) / np.linalg.norm(end - start)

        def rdp_recursive(points, epsilon):
            
            if len(points) < 3:
                return points

            start, end = points[0], points[-1]
            dmax, index = max((perpendicular_distance(points[i], start, end), i) for i in range(1, len(points) - 1))

            if dmax > epsilon:
                left = rdp_recursive(points[:index + 1], epsilon)
                right = rdp_recursive(points[index:], epsilon)
                return np.vstack((left[:-1], right))
            else:
                return np.array([start, end])

        return rdp_recursive(path, epsilon)

    def _remove_collinear_points(self, path):
        """
        Removing points that lie on a straight line using a collinearity check

        """
        waypoints = [path[0]]
        for i in range(1, len(path) - 1):
            prev, curr, next_ = waypoints[-1], path[i], path[i + 1]
            if not np.allclose(np.cross(curr - prev, next_ - curr), 0, atol=1e-3):
                waypoints.append(curr)
        waypoints.append(path[-1])
        return np.array(waypoints)

    def _compute_segment_times(self):
        """
        Computeing time allocation for each segment based on adaptive velocity selection.

        """
        fast_vel = 2  #  max speed
        moderate_vel = 2 #moderate speed
        slow_vel = 2 # Slowest speed

        times = [0]
        for i in range(1, len(self.points)):
            dist = np.linalg.norm(self.points[i] - self.points[i - 1])
            if dist > 3.2:
                velocity = fast_vel
            elif dist > 1.2:
                velocity = moderate_vel
            else:
                velocity = slow_vel
            times.append(times[-1] + dist / velocity)
        return np.array(times)

    def _compute_smooth_trajectory(self):
        """
        Computeing a smooth trajectory through waypoints using cubic spline

        """
        self.splines = {axis: CubicSpline(self.segment_times, self.points[:, i], bc_type='clamped')
                        for i, axis in enumerate(['x', 'y', 'z'])}
        
        # self.splines = {}  
        # for i, axis in enumerate(['x', 'y', 'z']):
        #     self.splines[axis] = make_interp_spline(self.segment_times, self.points[:, i], k=5)



    def update(self, t):
        """
        Computes desired trajectory state at time t.

        Parameters:
            t: Time in seconds.
        
        Returns:
            flat_output: Dictionary containing desired position, velocity, acceleration, jerk, and yaw.
        """
        if t <= self.segment_times[0]:  
            pos = self.points[0]
            vel = acc = jerk = np.zeros(3)
        elif t >= self.segment_times[-1]:  
            pos = self.points[-1]
            vel = acc = jerk = np.zeros(3)
        else:
            pos = np.array([self.splines[axis](t) for axis in ['x', 'y', 'z']])
            vel = np.array([self.splines[axis].derivative()(t) for axis in ['x', 'y', 'z']])
            acc = np.array([self.splines[axis].derivative(nu=2)(t) for axis in ['x', 'y', 'z']])
            jerk = np.array([self.splines[axis].derivative(nu=3)(t) for axis in ['x', 'y', 'z']])

        # Keeping yaw aligned with velocity direction
        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 1e-3 else 0
        # Assuming no yaw rotation
        yaw_dot = 0  

        return {
            'x': pos,
            'x_dot': vel,
            'x_ddot': acc,
            'x_dddot': jerk,
            'x_ddddot': np.zeros(3),
            'yaw': yaw,
            'yaw_dot': yaw_dot
        }
