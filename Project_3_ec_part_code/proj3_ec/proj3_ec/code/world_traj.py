import numpy as np

from .graph_search import graph_search
from ..util.occupancy_map import OccupancyMap
from .convert_path_to_waypoints import convert_path_to_waypoints
from scipy.interpolate import CubicSpline


class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal, local=False):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world,       World object representing the environment obstacles
            start,       xyz position in meters, shape=(3,)
            goal,        xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.3
        self.vmax = 2
        self.goal_tolerance = 0.5
        self.local_goal_list = []
        self.waypoints_list = []

        # Don't change this part
        self.local = local
        self.local_occ_map = OccupancyMap(world, self.resolution, self.margin)
        

        # The local map range is 5 m, usually the local planning range could be 1.5 * map range
        self.planning_horizon = 7.5 # m
        self.stopping_distance = 0.5 # m
        self.step_t = 0.02 # s
        self.no_replan_thresh = 2.0 # m

        
        # switch between replanning mode or execute trajectory mode.
        self.exec_traj = True
        # The collision checking frequency is usually higher than replanning frequency.
        # You may need to check it
        self.replan_num = 0
        self.t_check_traj = 0.1


        # generate init path
        self.global_goal  = np.asarray(goal,  dtype=float)
        self.start = np.asarray(start, dtype=float)
        self.traj_start_time = 0.0

        self.local_occ_map.update(self.start)
        self.plan_traj(self.start, self.crop_local_goal(start))




    def check_traj_collsion(self, cur_t):
        """
        Given current time, return the collision time
        Input:
        cur_t, absolute time or relative time, s 
        """
        check_t = cur_t
      
        while check_t < self.traj_start_time + self.traj_duration:

            check_t  += self.step_t
            check_pt  = self.get_traj_pos(check_t)
            if self.local_occ_map.is_occupied_metric(check_pt):
                return check_t
        check_t = -1
        return check_t
    

    def get_traj_pos(self, t):
        """
        Given the present time, return the desired position.

        Inputs
            t,   absolute time or relative time, s 
        Outputs

            x,   position, m
        """

        #TODO: Your code here. 

        tau = np.clip(t - self.traj_start_time, 0.0, self.traj_duration)
        x = np.array([
            self.spline_x(tau),
            self.spline_y(tau),
            self.spline_z(tau)
        ])
        return x


    def replan(self, cur_state, t):
        """
        Example framework for local planner. It can switch between replanning mode 
        or execute trajectory mode. You can use or improve it.
        
        Inputs
            cur_state,      a dict describing the state history with keys
                            x, position, m, shape=(N,3)
                            v, linear velocity, m/s, shape=(N,3)
                            q, quaternion [i,j,k,w], shape=(N,4)
                            w, angular velocity, rad/s, shape=(N,3)
            t,              absolute time, s 
        """
        cur_state = cur_state['x']
        # update map with new center point
        self.local_occ_map.update(cur_state)
        
        self.replan_num += 1
        # Check if replanning because of potential collision

        # TODO: you can set absolute time(t) or relative time as input, default it's absolute time
        imminent_collision_time = self.check_traj_collsion(t)
        need_replan = (imminent_collision_time>0 and imminent_collision_time <= t + 0.5 * self.traj_duration)
        
        # do not replan
        if self.replan_num < 20:
            if not need_replan:
                print("No need for replanning, the check_t is : ", imminent_collision_time)
                return 
        self.replan_num = 0

        
        if self.exec_traj: # exec mode
            # 1. reaching end, no plan threshld
            if np.linalg.norm(cur_state - self.global_goal) < self.stopping_distance: 
                print("Reaching end ...")
                return
            # 2. check no planning thresh
            if np.linalg.norm(self.start - cur_state) < self.no_replan_thresh:
                return
            self.exec_traj = False # transfer to replanning mode

        else: # replanning mode
            if t < self.traj_start_time + self.traj_duration: # redo planning
                cur_state = self.get_traj_pos(t)

            self.start = cur_state
            self.traj_start_time = t
            if self.plan_traj(self.start, self.crop_local_goal(self.start)):
                self.exec_traj = True
        return


    # def crop_local_goal(self, start):
    #     """
    #     Given local start, get a straight line position as the cropped local goal 
    #     and end up with the global goal
        
    #     Inputs
    #         start, xyz position in meters, shape=(3,)        
        
    #     Outputs
    #         goal,  xyz position in meters, shape=(3,)  
    #     """

    #     # You can also improve this function with local goal selection strategies

    #     dist = np.linalg.norm(self.global_goal - start)
    #     if dist <= self.planning_horizon:
    #         print("reaching global goal!")
    #         goal = self.global_goal
    #     else:
    #         # local goal here
    #         goal = start + (self.planning_horizon / dist)  * (self.global_goal - start)
    #     return goal


    def crop_local_goal(self, start):
        """
        Generate a cropped local goal that is guaranteed to be inside the current local map.
        
        Inputs:
            start: Current position, shape=(3,)
        Output:
            goal: Cropped local goal, shape=(3,)
        """

        dist = np.linalg.norm(self.global_goal - start)
        
        # First try to use full planning horizon
        planning_range = self.planning_horizon

        # Clamp to global goal if it's already close enough
        if dist <= planning_range:
            print("reaching global goal!")
            goal = self.global_goal
        else:
            # Try to shrink horizon until goal is inside local map bounds
            direction = (self.global_goal - start) / dist
            for _ in range(10):  # Try up to 10 times
                goal = start + direction * planning_range
                if self.local_occ_map.is_in_bounds_metric(goal):
                    return goal
                planning_range *= 0.8  # Shrink distance
            # Fallback: return current position (safe but not moving)
            print("[crop_local_goal] Failed to find valid goal in map, fallback to current pos")
            goal = start

        return goal

    def plan_traj(self, start:np.ndarray, goal:np.ndarray):
        """
        Given local start and goal, update the trajectory

        Inputs
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)
        
        """
        # TODO Your code here

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        path,node_expanded = graph_search(self.local_occ_map, self.resolution, self.margin, start, goal, self.goal_tolerance,astar=True)

        if path is None or len(path) < 2:
            print("[plan_traj] A* failed — no path to local goal")
            return False
        self.path = path
        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = convert_path_to_waypoints(self.path, self.resolution, self.margin)
        if np.linalg.norm(self.points[0] - start) > 1e-6:
            self.points = np.vstack([start, self.points])
        if np.linalg.norm(self.points[-1] - goal) > 1e-6:
            self.points = np.vstack([self.points, goal])

        self.N = len(self.points)
        if self.N < 2:
            print("[plan_traj] Not enough waypoints, force two points (start-goal)")
            self.points = np.vstack([start, goal])
            self.N = 2

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # You may need to update some trajectory information for replanning updates
        seg_times = []
        for i in range(self.N - 1):
            d = np.linalg.norm(self.points[i+1] - self.points[i])
            T = max(0.1, d / self.vmax)  # prevent too small times
            seg_times.append(T)
        self.segment_times = np.array(seg_times)
        self.time = np.concatenate(([0.0], np.cumsum(self.segment_times)))
        self.traj_duration = float(self.time[-1])  # update duration
        print(f"[plan_traj] New traj: {self.N} waypoints, duration = {self.traj_duration:.2f}s")

        t = self.time
        pts = self.points
        self.spline_x = CubicSpline(t, pts[:,0], bc_type='natural')
        self.spline_y = CubicSpline(t, pts[:,1], bc_type='natural')
        self.spline_z = CubicSpline(t, pts[:,2], bc_type='natural')
        
        self.local_goal = goal
        self.local_goal_list.append(goal) 
        self.waypoints_list.append(self.points.copy())
        return True


    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, absolute time, s 
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

        tau = np.clip(t - self.traj_start_time, 0, self.traj_duration)

        x = np.array([
            self.spline_x(tau),
            self.spline_y(tau),
            self.spline_z(tau)
        ])

        x_dot = np.array([
            self.spline_x.derivative(1)(tau),
            self.spline_y.derivative(1)(tau),
            self.spline_z.derivative(1)(tau)
        ])

        x_ddot = np.array([
            self.spline_x.derivative(2)(tau),
            self.spline_y.derivative(2)(tau),
            self.spline_z.derivative(2)(tau)
        ])

        x_dddot = np.array([
            self.spline_x.derivative(3)(tau),
            self.spline_y.derivative(3)(tau),
            self.spline_z.derivative(3)(tau)
        ])

        # snap
        x_ddddot = np.array([
            self.spline_x.derivative(4)(tau),
            self.spline_y.derivative(4)(tau),
            self.spline_z.derivative(4)(tau)
        ])

        # yaw 取速度方向
        # if np.linalg.norm(x_dot[:2]) > 1e-3:
        #     yaw = np.arctan2(x_dot[1], x_dot[0])
        # else:
        yaw = 0
        yaw_dot = 0

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
