import numpy as np
import cvxopt
import cvxopt.solvers
from .graph_search import graph_search
from .convert_path_to_waypoints import convert_path_to_waypoints

cvxopt.solvers.options['show_progress'] =False
cvxopt.solvers.options['abstol'] = 1e-8
cvxopt.solvers.options['reltol'] = 1e-6


class WorldTraj(object):
    def __init__(self, world, start, goal):
        self.resolution = np.array([0.1, 0.1, 0.1])
        distance = np.linalg.norm(np.array(goal) - np.array(start))
        print(f"distance: {distance}m")
        # print(f"distance: {distance}m")
        if  18< distance < 19 :  
            self.margin = 0.5
            self.v_max =  1.5
            print(f"long({distance:.2f}m): use  margin={self.margin}, v_max={self.v_max}")
        # elif  21< distance:
        #     self.margin = 0.9
        #     self.v_max =  3.1
        else:
            self.margin = 0.53
            self.v_max = 1.9

        #     self.margin = 0.5 
        #     self.v_max = 3
        #     print(f"stand({distance:.2f}m): use margin={self.margin}, v_max={self.v_max}")
        # else:
        # self.margin = 0.5
        # self.v_max =2.9

        # self.margin = 0.5
        # self.v_max = 3
        # self.a_max = 7
        # self.j_max = 8.0
        self.time_opt_iterations = 5
        self.time_opt_step_size = 0.01
        self._plan_path(world, start, goal)
        self._optimize_time_allocation()
        self._solve_minimum_jerk_trajectory()
        
    def _plan_path(self, world, start, goal):
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        if self.path is None:
            raise ValueError('Could not find a path from start to goal')
        self.path = np.array(self.path)
        
        self.waypoint_traj = convert_path_to_waypoints(self.path, self.resolution, self.margin)
        
        if np.linalg.norm(self.waypoint_traj[0] - start) > 1e-6:
            self.waypoint_traj = np.vstack([start, self.waypoint_traj])
            
        if np.linalg.norm(self.waypoint_traj[-1] - goal) > 1e-6:
            self.waypoint_traj = np.vstack([self.waypoint_traj, goal])
            
        self.points = self.waypoint_traj
        self.N = self.points.shape[0]
        
        if self.N < 2:
            print("Warning: Too few path points, using original path points")
            self.points = np.vstack([start, goal])
            self.N = self.points.shape[0]
        
        print(f"Path planning complete: Simplified from {len(self.path)} points to {self.N} waypoints")
    
    def _initial_time_allocation(self):
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
    
    def _optimize_time_allocation(self):
        """
        Optimize time allocation using gradient descent - Section V.C of Mellinger's paper
        
        This method iteratively optimizes the time allocation for each trajectory segment
        to minimize the overall jerk.
        """
        self._initial_time_allocation()
        
        if self.N <= 2:
            return
        
        # print("Starting time allocation optimization...")
        
        for iteration in range(self.time_opt_iterations):
            self._compute_derivatives()
            cost_before = self._evaluate_trajectory_cost()
            
            gradients = np.zeros(self.N - 1)
            delta_t = 0.001
            
            for i in range(self.N - 1):
                self.segment_times[i] += delta_t
                self.time = np.concatenate(([0], np.cumsum(self.segment_times)))
                self.tf = self.time[-1]
                
                self._compute_derivatives()
                cost_after = self._evaluate_trajectory_cost()
                
                self.segment_times[i] -= delta_t
                
                gradients[i] = (cost_after - cost_before) / delta_t
            
            step_size = self.time_opt_step_size / (1 + 0.1 * iteration)
            self.segment_times -= step_size * gradients
            
            self.segment_times = np.maximum(0.1, self.segment_times)
            
            self.time = np.concatenate(([0], np.cumsum(self.segment_times)))
            self.tf = self.time[-1]
            
            self._compute_derivatives()
            new_cost = self._evaluate_trajectory_cost()
            improvement = (cost_before - new_cost) / cost_before * 100 if cost_before > 0 else 0
            
            # print(f"  Iteration {iteration+1}/{self.time_opt_iterations}: Cost {new_cost:.4f}, Improvement {improvement:.2f}%")
            
            if improvement < 0.1:
                break
        
        # print(f"Time optimization complete: Total trajectory time {self.tf:.2f} seconds")
    
    def _compute_derivatives(self):
        self.v = np.zeros((self.N, 3))
        self.a = np.zeros((self.N, 3))
        
        if self.N <= 1:
            return
            
        self.v[0, :] = np.zeros(3)
        self.v[-1, :] = np.zeros(3)
        
        for i in range(1, self.N - 1):
            dist_prev = np.linalg.norm(self.points[i] - self.points[i-1])
            dist_next = np.linalg.norm(self.points[i+1] - self.points[i])
            
            if dist_prev > 1e-6 and dist_next > 1e-6:
                dir_prev = (self.points[i] - self.points[i-1]) / dist_prev
                dir_next = (self.points[i+1] - self.points[i]) / dist_next
                
                smooth_dir = (dir_prev * dist_next + dir_next * dist_prev) / (dist_prev + dist_next)
                smooth_dir = smooth_dir / np.linalg.norm(smooth_dir) if np.linalg.norm(smooth_dir) > 1e-6 else smooth_dir
                
                dot_product = np.clip(np.dot(dir_prev, dir_next), -1.0, 1.0)
                angle = np.arccos(dot_product)
                speed = self.v_max * (1.0 - 0.5 * angle / np.pi)
                
                self.v[i, :] = smooth_dir * speed
    
    def _evaluate_trajectory_cost(self):
        total_cost = 0
        
        for i in range(self.N - 1):
            segment_time = self.segment_times[i]
            if segment_time > 1e-6:
                if i < self.N - 2:
                    acc_change = np.linalg.norm(self.a[i+1] - self.a[i])
                    jerk_approx = acc_change / segment_time
                    total_cost += jerk_approx**2 * segment_time
        
        return total_cost

    def _setup_constraints(self, dim):
        n = self.N - 1
        d = 6
        if n == 0:
            return None, None, None, None, None, None
        P = np.zeros((d*n, d*n))
        q = np.zeros(d*n)
        
        for i in range(n):
            dt = self.segment_times[i]
            P_i = np.zeros((d, d))
            
            T = dt
            P_i[3, 3] = 36 * T
            P_i[3, 4] = 72 * T**2
            P_i[3, 5] = 120 * T**3
            
            P_i[4, 3] = 72 * T**2
            P_i[4, 4] = 192 * T**3
            P_i[4, 5] = 360 * T**4
            
            P_i[5, 3] = 120 * T**3
            P_i[5, 4] = 360 * T**4
            P_i[5, 5] = 720 * T**5
            
            P[i*d:(i+1)*d, i*d:(i+1)*d] = P_i
        
        n_constraints = n + 1 + 4 + 3*(n-1)
        A = np.zeros((n_constraints, d*n))
        b = np.zeros(n_constraints)
        
        constraint_idx = 0
        
        for i in range(n + 1):
            if i == 0:
                A[constraint_idx, 0] = 1
                b[constraint_idx] = self.points[0, dim]
                constraint_idx += 1
            elif i == n:
                dt = self.segment_times[n-1]
                A[constraint_idx, (n-1)*d + 0] = 1
                A[constraint_idx, (n-1)*d + 1] = dt
                A[constraint_idx, (n-1)*d + 2] = dt**2
                A[constraint_idx, (n-1)*d + 3] = dt**3
                A[constraint_idx, (n-1)*d + 4] = dt**4
                A[constraint_idx, (n-1)*d + 5] = dt**5
                b[constraint_idx] = self.points[n, dim]
                constraint_idx += 1
            else:
                dt = self.segment_times[i-1]
                A[constraint_idx, (i-1)*d + 0] = 1
                A[constraint_idx, (i-1)*d + 1] = dt
                A[constraint_idx, (i-1)*d + 2] = dt**2
                A[constraint_idx, (i-1)*d + 3] = dt**3
                A[constraint_idx, (i-1)*d + 4] = dt**4
                A[constraint_idx, (i-1)*d + 5] = dt**5
                b[constraint_idx] = self.points[i, dim]
                constraint_idx += 1
        
        A[constraint_idx, 1] = 1
        b[constraint_idx] = self.v[0, dim]
        constraint_idx += 1
        A[constraint_idx, 2] = 2
        b[constraint_idx] = self.a[0, dim]
        constraint_idx += 1
        dt = self.segment_times[n-1]
        A[constraint_idx, (n-1)*d + 1] = 1
        A[constraint_idx, (n-1)*d + 2] = 2*dt
        A[constraint_idx, (n-1)*d + 3] = 3*dt**2
        A[constraint_idx, (n-1)*d + 4] = 4*dt**3
        A[constraint_idx, (n-1)*d + 5] = 5*dt**4
        b[constraint_idx] = self.v[n, dim]
        constraint_idx += 1
        
        A[constraint_idx, (n-1)*d + 2] = 2
        A[constraint_idx, (n-1)*d + 3] = 6*dt
        A[constraint_idx, (n-1)*d + 4] = 12*dt**2
        A[constraint_idx, (n-1)*d + 5] = 20*dt**3
        b[constraint_idx] = self.a[n, dim]
        constraint_idx += 1
        
        for i in range(n-1):
            dt = self.segment_times[i]
            
            A[constraint_idx, i*d + 0] = 1
            A[constraint_idx, i*d + 1] = dt
            A[constraint_idx, i*d + 2] = dt**2
            A[constraint_idx, i*d + 3] = dt**3
            A[constraint_idx, i*d + 4] = dt**4
            A[constraint_idx, i*d + 5] = dt**5
            A[constraint_idx, (i+1)*d + 0] = -1
            b[constraint_idx] = 0
            constraint_idx += 1
            
            A[constraint_idx, i*d + 1] = 1
            A[constraint_idx, i*d + 2] = 2*dt
            A[constraint_idx, i*d + 3] = 3*dt**2
            A[constraint_idx, i*d + 4] = 4*dt**3
            A[constraint_idx, i*d + 5] = 5*dt**4
            A[constraint_idx, (i+1)*d + 1] = -1
            b[constraint_idx] = 0
            constraint_idx += 1
            
            A[constraint_idx, i*d + 2] = 2
            A[constraint_idx, i*d + 3] = 6*dt
            A[constraint_idx, i*d + 4] = 12*dt**2
            A[constraint_idx, i*d + 5] = 20*dt**3
            A[constraint_idx, (i+1)*d + 2] = -2
            b[constraint_idx] = 0
            constraint_idx += 1
        
        use_velocity_constraints = True
        
        if use_velocity_constraints:
            samples_per_segment = 3
            total_samples = samples_per_segment * n
            
            G = np.zeros((2 * total_samples, d*n))
            h = np.ones(2 * total_samples) * self.v_max
            
            constraint_idx = 0
            for i in range(n):
                dt = self.segment_times[i]
                for j in range(samples_per_segment):
                    t = j * dt / (samples_per_segment - 1) if samples_per_segment > 1 else 0
                    
                    G[constraint_idx, i*d + 1] = 1
                    G[constraint_idx, i*d + 2] = 2*t
                    G[constraint_idx, i*d + 3] = 3*t**2
                    G[constraint_idx, i*d + 4] = 4*t**3
                    G[constraint_idx, i*d + 5] = 5*t**4
                    
                    G[constraint_idx + total_samples, i*d + 1] = -1
                    G[constraint_idx + total_samples, i*d + 2] = -2*t
                    G[constraint_idx + total_samples, i*d + 3] = -3*t**2
                    G[constraint_idx + total_samples, i*d + 4] = -4*t**3
                    G[constraint_idx + total_samples, i*d + 5] = -5*t**4
                    
                    constraint_idx += 1
        else:
            G, h = None, None
            
    
        use_corridor_constraints = True  
        # A = 0.2
        if use_corridor_constraints:
        
            corridor_margin = 0.05

            samples_per_segment = 5

            total_samples = n * samples_per_segment * 2

            G_corridor = np.zeros((total_samples, d*n))
            h_corridor = np.zeros(total_samples)

            corridor_idx = 0

            for i in range(n):
                dt = self.segment_times[i]

                p0 = self.points[i, dim]
                p1 = self.points[i+1, dim]
                corridor_min = min(p0, p1) - corridor_margin
                corridor_max = max(p0, p1) + corridor_margin
                for s in range(samples_per_segment):
            
                    t_s = (dt * s) / (samples_per_segment - 1) if samples_per_segment>1 else 0.0
                    G_corridor[corridor_idx, i*d + 0] = 1
                    G_corridor[corridor_idx, i*d + 1] = t_s
                    G_corridor[corridor_idx, i*d + 2] = t_s**2
                    G_corridor[corridor_idx, i*d + 3] = t_s**3
                    G_corridor[corridor_idx, i*d + 4] = t_s**4
                    G_corridor[corridor_idx, i*d + 5] = t_s**5
                    h_corridor[corridor_idx] = corridor_max
                    corridor_idx += 1

                    G_corridor[corridor_idx, i*d + 0] = -1
                    G_corridor[corridor_idx, i*d + 1] = -t_s
                    G_corridor[corridor_idx, i*d + 2] = -(t_s**2)
                    G_corridor[corridor_idx, i*d + 3] = -(t_s**3)
                    G_corridor[corridor_idx, i*d + 4] = -(t_s**4)
                    G_corridor[corridor_idx, i*d + 5] = -(t_s**5)
                    h_corridor[corridor_idx] = -corridor_min
                    corridor_idx += 1

            G = G_corridor
            h = h_corridor
        else:
            G, h = None, None

        return A, b, G, h, P, q
    
    def _solve_minimum_jerk_trajectory(self):
        self.seg = []
        
        if self.N <= 1:
            return
            
        # print("Starting minimum-jerk trajectory optimization...")
        
        for dim in range(3):
            A, b, G, h, P, q = self._setup_constraints(dim)
            
            if A is None:
                continue
                
            P_cvx = cvxopt.matrix(P)
            q_cvx = cvxopt.matrix(q)
            A_cvx = cvxopt.matrix(A)
            b_cvx = cvxopt.matrix(b)
            
            if G is not None and h is not None:
                G_cvx = cvxopt.matrix(G)
                h_cvx = cvxopt.matrix(h)
                sol = cvxopt.solvers.qp(P_cvx, q_cvx, G_cvx, h_cvx, A_cvx, b_cvx)
            else:
                sol = cvxopt.solvers.qp(P_cvx, q_cvx, A=A_cvx, b=b_cvx)
            
            if sol['status'] != 'optimal':
                print(f"Warning: Dimension {dim} QP solution did not reach optimality, using fallback method")
                try:
                    coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
                except np.linalg.LinAlgError:
                    print(f"Warning: Dimension {dim} least squares solution failed, using simple interpolation")
                    coeffs = np.zeros(A.shape[1])
                    for i in range(self.N - 1):
                        p0 = self.points[i, dim]
                        p1 = self.points[i+1, dim]
                        dt = self.segment_times[i]
                        coeffs[i*6] = p0
                        coeffs[i*6+1] = (p1-p0)/dt if dt > 0 else 0
            else:
                coeffs = np.array(sol['x']).flatten()
                
            for i in range(self.N - 1):
                if i >= len(self.seg):
                    self.seg.append({
                        't0': self.time[i],
                        'tf': self.time[i+1],
                        'dt': self.segment_times[i],
                        'coeffs': np.zeros((6, 3))
                    })
                
                self.seg[i]['coeffs'][:, dim] = coeffs[i*6:(i+1)*6]
        
        print("Trajectory optimization complete")
        
        self._verify_trajectory()
    
    def _verify_trajectory(self):
        max_error = 0
        for i in range(self.N):
            if i == 0:
                t = 0
            elif i == self.N - 1:
                t = self.tf
            else:
                t = self.time[i]
            
            flat_output = self.update(t)
            error = np.linalg.norm(flat_output['x'] - self.points[i])
            max_error = max(max_error, error)
            
            if error > 0.01:
                print(f"Warning: Error at waypoint {i} is {error:.6f} meters")
        
        if max_error < 0.01:
            print(f"Trajectory verification passed: Maximum error {max_error:.6f} meters")
        else:
            print(f"Trajectory verification result: Maximum error {max_error:.6f} meters")

    def update(self, t):
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
            t = min(max(0, t), self.tf)
            
            if t >= self.tf:
                which_seg = self.N - 2
                time_in_seg = t - self.time[which_seg]
            else:
                which_seg = np.searchsorted(self.time[1:], t, side='right')
                time_in_seg = t - self.time[which_seg]
            
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


