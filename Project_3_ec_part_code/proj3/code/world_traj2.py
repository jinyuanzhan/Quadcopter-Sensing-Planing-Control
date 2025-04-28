import numpy as np

from .graph_search2 import graph_search
from .occupancy_map import OccupancyMap
from scipy.linalg import block_diag, solve

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        # self.resolution = np.array([0.1, 0.1, 0.1])
        # self.margin = 0.3
        self.resolution = np.array([0.1, 0.1, 0.1])
        self.margin = 0.55
        # self.occupancy_map = OccupancyMap(world, self.resolution, self.margin)

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = self.rdp_waypoints_simp(self.path, eps=0.1) # shape=(n_pts,3)
        print(self.points)


        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.
        self.times = self.compute_segment_times(self.points, 1.0) # shape=(M,)
        self.times[-1] *= 3.0
        self.times[0] *= 2.0
        # Set time checkpoints to determine the segment 
        self.checkpoints = np.cumsum(self.times) # shape=(M,)
        self.checkpoints[-1] = np.inf
        self.next_checkpoint = 0

        print("Number of waypoints:", len(self.points))
        print("Number of times:", len(self.times))
        self.M = len(self.times) 
        self.N = 3 # The order of the polynomial
        self.num_coef = self.N + 1

        self.seg_idx = 0
        self.t_cum_seg = 0.0

        Q_blocks = []
        for T_i in self.times:
            Q_blocks.append(self.compute_quadratic_Q_matrix(T_i, self.N))
        Q_total = block_diag(*Q_blocks)

        self.coeffs = {}
        for dim, axis in enumerate(['x', 'y', 'z']):
            waypoint_1d = self.points[:, dim]
            A, b = self.compute_constraint_matrix(self.times, waypoint_1d, self.N)
            n_vars = Q_total.shape[0]
            n_cons = A.shape[0]
            
            # KKT system
            KKT = np.block([
                [Q_total, A.T],
                [A, np.zeros((n_cons, n_cons))]
            ])
            rhs = np.concatenate([np.zeros(n_vars), b])
            sol = solve(KKT, rhs)
            c = sol[:n_vars]  # polynomial coefficients for this dimension
            # Reshape
            self.coeffs[axis] = [c[i*self.num_coef:(i+1)*self.num_coef] for i in range(self.M)]
        
        self.total_time = np.sum(self.times)
        print(f"self.times:{self.times}")
        print(f"self.total_time: {self.total_time}")


        # STUDENT CODE HERE


    def rdp_waypoints_simp(self, points: np.ndarray, eps):
        """
        Simplify a path of waypoints using the Ramer-Douglas-Peucker algorithm.

        Parameters:
            points, np.ndarray, shape=(n_pts,3)
            eps, float, maximum distance from a point to the line between the endpoints

        Returns:
            np.ndarray, shape=(n_simp,3), the simplified path
        """
        # Sanity check: if there are less than 3 points, we return directly:
        if len(points) < 3:
            return points
        # We will recursively simplify the waypoints based on the largest deviation and the tolerance eps:
        # First record the start and end:
        start = points[0]
        end = points[-1]

        max_dis = -1.0
        index_max = 0

        for i in range(1, len(points)-1):
            distance = self.perpendicular_dist(start, end, points[i])
            if distance > max_dis:
                max_dis = distance
                index_max = i

        if max_dis >= eps: 
            # In this case, there are still important intermediate points, we do the simplification recursively
            points_left = self.rdp_waypoints_simp(points[:index_max+1], eps)
            points_right = self.rdp_waypoints_simp(points[index_max:], eps)

            return np.vstack((points_left[:-1], points_right))
        elif np.linalg.norm(start-end) >= 2:
            return np.array([start, start+ (end -start)/3, start+ 2 * (end -start)/3,  end])
        else:
            return np.array([start, end])
    
    def perpendicular_dist(self, line_start, line_end, point):
        # Calculate the perpendicular distance from the point to a line.
        # First vectorize the three points
        line_start = np.array(line_start)
        line_end = np.array(line_end)
        point = np.array(point)

        line = line_end - line_start
        start2point = point - line_start

        if np.allclose(line, 0):
        # If line_start and line_end are the same, distance is direct
            return np.linalg.norm(start2point)

        # Project the start_point line to the target line:
        projection = np.dot(line, start2point) / np.dot(line, line)
        # Find the projection point on the target line:
        nearest_pt = line_start + projection * line

        # Calculate the perpendicular distance:
        distance = np.linalg.norm(nearest_pt - point)
        return distance
    
    def compute_segment_times(self, points, alpha = 3.5):
        # Compute the distance adaptive timepoint for each segment
        # points are the simplified waypoints returned by the rdp algorithm
        # We want our speed be proportional to the disctance between two adjacent waypoints, alpha defines a proportion
        # If there are less than two poins, return nothing
        if len(points)<2:
            return np.array([])
        
        dist = np.linalg.norm(np.diff(points, axis=0), axis=1)
        print(dist)
        vels = dist * alpha
        vels = np.clip(vels, 2.3, 3.6)
        for i, d in enumerate(dist):
            if d > 8.1:
                vels[i] = 7.0
        print(vels)
        T = dist/vels
        return T
    
    def compute_quadratic_Q_matrix(self, T, N):
        # Compute the Q matrix for the quadratic cost of each segment
        # For the minimum acceleration, the polynomial order N = 3
        # Only c3 and c2 will left in the cost after taking the second derivative. Other entries in Q will be 0
        Q = np.zeros((4, 4))
        for i in range(2, N+1):
            for j in range(2, N+1):
            # i*(i-1)* j*(j-1)* T^(i+j-3) / (i+j-3)
                factor = (i*(i-1))*(j*(j-1))
                power = (i + j - 3)
                denom = (i + j - 3)
                Q[i, j] = factor * T**power / denom
        return Q
    
    def compute_constraint_matrix(self, times, waypoints, N):
        # Construct constraints as Ax = b
        A_list = [] # A should be a num_constraints * num_var matrix
        b_list = [] 

        # Number of segments:
        M = len(times)
        num_coef = N + 1
        num_var = M * num_coef

        # Note that waypoints here is in 1D and we will deal with each x, y, z dimensions seperately


        # Position constraints:
        # At the beginning of each segment, the position should be at waypoints[i]
        for i in range(M):
            row_start = np.zeros(num_var)
            row_start[i*num_coef + 0] = 1.0 # The first coeffiecient of each segment, correspond to position
            A_list.append(row_start)
            b_list.append(waypoints[i])
            # print(f"Adding the segment start position constraint:{waypoints[i]}")

            row_end = np.zeros(num_var)
            for j in range(num_coef):
                row_end[i*num_coef + j] = times[i]**j
            A_list.append(row_end)
            b_list.append(waypoints[i+1])
            # print(f"Adding the segment end position constraint:{waypoints[i+1]}")

        # Velocity constraints:
        # first, the initial velocity and terminal velocity should be 0
        row_initial = np.zeros(num_var)
        row_initial[1] = 1.0
        A_list.append(row_initial)
        b_list.append(0.0)

        row_terminal = np.zeros(num_var)
        for j in range(1, num_coef):
            row_terminal[(M-1)*num_coef + j] = j* times[-1]**(j-1)
        A_list.append(row_terminal)
        b_list.append(0.0)

        # Then we contrain the velocity continuity, the velocity at the end of the i-1 segment should be the same as the begining of the i segment
        for i in range(1, M):
            row_continuity = np.zeros(num_var)
            for j in range(1, num_coef):
                # The end of the i-1 segment velocity
                row_continuity[(i-1)*num_coef + j] = j*times[i-1]**(j-1)
                # The start of the i segment velocity
            row_continuity[i*num_coef + 1] = -1.0

            A_list.append(row_continuity)
            b_list.append(0.0)
        
        A = np.vstack(A_list)
        b = np.array(b_list)
        return A, b
    
    def evaluate_cubic(self, coeffs, t):
        """
        For p(t)= a0 + a1 t + a2 t^2 + a3 t^3:
          p'(t)= a1 + 2 a2 t + 3 a3 t^2
          p''(t)= 2 a2 + 6 a3 t
          p'''(t)= 6 a3 (constant)
        Returns p, p_dot, p_ddot, p_dddot
        """
        a0, a1, a2, a3 = coeffs
        p = a0 + a1*t + a2*t**2 + a3*t**3
        p_dot = a1 + 2*a2*t + 3*a3*t**2
        p_ddot = 2*a2 + 6*a3*t
        p_dddot = 6*a3
        return p, p_dot, p_ddot, p_dddot









    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        if t < self.t_cum_seg:
            self.t_cum_seg = 0.0
            self.seg_idx = 0
            self.next_checkpoint = 0

        if t > self.total_time:
            t = self.total_time - 1e-6

        # Now use the while loop to determine the current segment.
        while t > self.checkpoints[self.next_checkpoint]:
            self.t_cum_seg = self.checkpoints[self.next_checkpoint]
            self.seg_idx += 1
            print(f"Switching to segment {self.seg_idx}")
            self.next_checkpoint += 1

        t_local_seg = t - self.t_cum_seg


        pos = np.zeros(3)
        vel = np.zeros(3)
        acc = np.zeros(3)
        jerk = np.zeros(3)   
        snap = np.zeros(3)

        for i, axis in enumerate(['x', 'y', 'z']):
            coeff = self.coeffs[axis][self.seg_idx]
            p, p_dot, p_ddot, p_dddot = self.evaluate_cubic(coeff, t_local_seg)
            pos[i] = p
            vel[i] = p_dot
            acc[i] = p_ddot
            jerk[i] = p_dddot

        x        = pos
        x_dot    = vel
        x_ddot   = acc
        x_dddot  = jerk
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # print(x)

        # STUDENT CODE HERE

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output