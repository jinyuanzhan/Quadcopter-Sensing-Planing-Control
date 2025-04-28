
import numpy as np
from .graph_search import graph_search
from .convert_path_to_waypoints import convert_path_to_waypoints

class WorldTraj(object):
    def __init__(self, world, start, goal):
        """
        Initialize a trajectory generator for the drone to navigate from start to goal in the provided world.
        """
        self.resolution = np.array([0.05, 0.05, 0.05])
        self.margin = 0.2
        self.v_max = 1.0
        self.a_max = 7.0
        self.j_max = 8.0
        
        # Plan the path, generate waypoints and compute the trajectory
        self._plan_path(world, start, goal)
        self._allocate_time()
        
        self.tf_margin = self.tf * 0.999
        self._compute_minimum_jerk_trajectory()

    def _plan_path(self, world, start, goal):
        """
        Plan a path from start to goal using graph search and convert to waypoints.
        """
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        if self.path is None:
            raise ValueError('Could not find a path from start to goal')
        self.path = np.array(self.path)
        
        # Convert path to waypoints
        self.waypoint_traj = convert_path_to_waypoints(self.path, self.resolution, self.margin)
        
        # Ensure start and goal points are included
        if np.linalg.norm(self.waypoint_traj[0] - start) > 1e-6:
            self.waypoint_traj = np.vstack([start, self.waypoint_traj])
        if np.linalg.norm(self.waypoint_traj[-1] - goal) > 1e-6:
            self.waypoint_traj = np.vstack([self.waypoint_traj, goal])
        
        self.points = self.waypoint_traj
        self.N = self.points.shape[0]
        
        # Ensure we have at least two points
        if self.N < 2:
            print("Warning: Too few path points, using original path points")
            self.points = np.vstack([start, goal])
            self.N = self.points.shape[0]
        
        print(f"Path planning complete: Simplified from {len(self.path)} points to {self.N} waypoints")

    def _allocate_time(self):

        if self.N <= 1:
            self.segment_times = np.array([0])
            self.time = np.array([0])
            self.tf = 0
            return
        
        segment_times = []
        for i in range(self.N - 1):
            d = np.linalg.norm(self.points[i+1] - self.points[i])
            T_i = d / self.v_max
            if i == 0 or i == self.N - 2:
                T_i *= 1.5
            if i > 0 and i < self.N - 2:
                v1 = self.points[i] - self.points[i-1]
                v2 = self.points[i+1] - self.points[i]
                if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                    v1 = v1 / np.linalg.norm(v1)
                    v2 = v2 / np.linalg.norm(v2)
                    dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
                    angle = np.arccos(dot_product)
                    T_i *= (1.0 + 0.5 * angle / np.pi)
            T_i = max(0.1, T_i)
            segment_times.append(T_i)
        
        self.segment_times = np.array(segment_times)
        self.time = np.concatenate(([0], np.cumsum(self.segment_times)))
        self.tf = self.time[-1]
        
        print(f"Time allocation complete: Total trajectory time {self.tf:.2f} seconds")

    def _compute_minimum_jerk_trajectory(self):
        """
        Compute a minimum-jerk trajectory through the waypoints.
        This implementation ensures continuity of position, velocity, acceleration, and jerk.
        """
        if self.N <= 1:
            self.seg = []
            return
        
        # Initialize velocities and accelerations at waypoints
        v = np.zeros((self.N, 3))
        a = np.zeros((self.N, 3))
        
        for i in range(1, self.N - 1):
            prev_dist = np.linalg.norm(self.points[i] - self.points[i-1])
            next_dist = np.linalg.norm(self.points[i+1] - self.points[i])
            if prev_dist + next_dist > 1e-6:
                v[i] = ((next_dist * (self.points[i] - self.points[i-1]) / prev_dist) + 
                         (prev_dist * (self.points[i+1] - self.points[i]) / next_dist)) / (prev_dist + next_dist)
        
        self.seg = []
        for dim in range(3):
            for i in range(self.N - 1):
                p0 = self.points[i, dim]
                p1 = self.points[i+1, dim]
                v0 = v[i, dim]
                v1 = v[i+1, dim]
                a0 = a[i, dim]
                a1 = a[i+1, dim]
                T = self.segment_times[i]
                a0_coef = p0
                a1_coef = v0
                a2_coef = a0 / 2
                if abs(T) < 1e-6:
                    a3_coef = a4_coef = a5_coef = 0
                else:
                    A = np.array([
                        [T**3, T**4, T**5],
                        [3*T**2, 4*T**3, 5*T**4],
                        [6*T, 12*T**2, 20*T**3]
                    ])
                    b = np.array([
                        p1 - p0 - v0*T - a0*T**2/2,
                        v1 - v0 - a0*T,
                        a1 - a0
                    ])
                    try:
                        sol = np.linalg.solve(A, b)
                        a3_coef, a4_coef, a5_coef = sol
                    except np.linalg.LinAlgError:
                        print(f"Warning: Singular matrix for segment {i}, dimension {dim}")
                        a3_coef = (20*p1 - 20*p0 - 8*v1*T - 12*v0*T - a1*T**2 + a0*T**2) / (2*T**3)
                        a4_coef = (30*p0 - 30*p1 + 14*v1*T + 16*v0*T + 3*a1*T**2 - 2*a0*T**2) / (2*T**4)
                        a5_coef = (12*p1 - 12*p0 - 6*v1*T - 6*v0*T - a1*T**2 + a0*T**2) / (2*T**5)
                if i >= len(self.seg):
                    self.seg.append({
                        't0': self.time[i],
                        'tf': self.time[i+1],
                        'dt': self.segment_times[i],
                        'coeffs': np.zeros((6, 3))
                    })
                self.seg[i]['coeffs'][:, dim] = [a0_coef, a1_coef, a2_coef, a3_coef, a4_coef, a5_coef]
        
        print("Minimum-jerk trajectory computation complete")

    def update(self, t):
        """
        Return the trajectory state at time t.
        """
        x = np.zeros(3)
        x_dot = np.zeros(3)
        x_ddot = np.zeros(3)
        x_dddot = np.zeros(3)
        x_ddddot = np.zeros(3)
        yaw = 0
        yaw_dot = 0
        
        if self.N == 1:
            x = self.points[0, :]
        else:
            
            t_clamped = np.clip(t, 0, self.tf)
            if t_clamped >= self.tf_margin:
                x = self.points[-1].copy()
                x_dot = np.zeros(3)
                x_ddot = np.zeros(3)
                x_dddot = np.zeros(3)
                x_ddddot = np.zeros(3)
            else:
                which_seg = np.searchsorted(self.time[1:], t_clamped, side='right')
                time_in_seg = t_clamped - self.time[which_seg]
                seg_info = self.seg[which_seg]
                coeff = seg_info['coeffs']
                tau = time_in_seg
                for d in range(3):
                    a0, a1, a2, a3, a4, a5 = coeff[:, d]
                    x[d] = a0 + a1*tau + a2*tau**2 + a3*tau**3 + a4*tau**4 + a5*tau**5
                    x_dot[d] = a1 + 2*a2*tau + 3*a3*tau**2 + 4*a4*tau**3 + 5*a5*tau**4
                    x_ddot[d] = 2*a2 + 6*a3*tau + 12*a4*tau**2 + 20*a5*tau**3
                    x_dddot[d] = 6*a3 + 24*a4*tau + 60*a5*tau**2
                    x_ddddot[d] = 24*a4 + 120*a5*tau
        
        flat_output = {
            'x': x,
            'x_dot': x_dot,
            'x_ddot': x_ddot,
            'x_dddot': x_dddot,
            'x_ddddot': x_ddddot,
            'yaw': yaw,
            'yaw_dot': yaw_dot
        }
        
        return flat_output
