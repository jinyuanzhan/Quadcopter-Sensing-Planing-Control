# import numpy as np

# from .graph_search import graph_search
# from ..util.occupancy_map import OccupancyMap

# class WorldTraj(object):
#     """

#     """
#     def __init__(self, world, start, goal, local=False):
#         """
#         This is the constructor for the trajectory object. A fresh trajectory
#         object will be constructed before each mission. For a world trajectory,
#         the input arguments are start and end positions and a world object. You
#         are free to choose the path taken in any way you like.

#         You should initialize parameters and pre-compute values such as
#         polynomial coefficients here.

#         Parameters:
#             world,       World object representing the environment obstacles
#             start,       xyz position in meters, shape=(3,)
#             goal,        xyz position in meters, shape=(3,)

#         """

#         # You must choose resolution and margin parameters to use for path
#         # planning. In the previous project these were provided to you; now you
#         # must chose them for yourself. Your may try these default values, but
#         # you should experiment with them!
#         self.resolution = np.array([0.25, 0.25, 0.25])
#         self.margin = 0.5

#         # Don't change this part
#         self.local = local
#         self.local_occ_map = OccupancyMap(world, self.resolution, self.margin)
        

#         # The local map range is 5 m, usually the local planning range could be 1.5 * map range
#         self.planning_horizon = 7.5 # m
#         self.stopping_distance = 0.5 # m
#         self.step_t = 0.02 # s
#         self.no_replan_thresh = 2.0 # m

        
#         # switch between replanning mode or execute trajectory mode.
#         self.exec_traj = True
#         # The collision checking frequency is usually higher than replanning frequency.
#         # You may need to check it
#         self.replan_num = 0
#         self.t_check_traj = 0.1


#         # generate init path
#         self.global_goal  = goal
#         self.start = start
#         self.traj_start_time = 0.0

#         self.local_occ_map.update(self.start)
#         self.plan_traj(self.start, self.crop_local_goal(start))




#     def check_traj_collsion(self, cur_t):
#         """
#         Given current time, return the collision time
#         Input:
#         cur_t, absolute time or relative time, s 
#         """
#         check_t = cur_t
      
#         while check_t < self.traj_start_time + self.traj_duration:

#             check_t  += self.step_t
#             check_pt  = self.get_traj_pos(check_t)

#             if self.local_occ_map.is_occupied_metric(check_pt):
#                 return check_t

#         check_t = -1
#         return check_t
    

#     def get_traj_pos(self, t):
#         """
#         Given the present time, return the desired position.

#         Inputs
#             t,   absolute time or relative time, s 
#         Outputs

#             x,   position, m
#         """

#         #TODO: Your code here. 


#         x        = np.zeros((3,))

#         return x


      
#     def replan(self, cur_state, t):
#         """
#         Example framework for local planner. It can switch between replanning mode 
#         or execute trajectory mode. You can use or improve it.
        
#         Inputs
#             cur_state,      a dict describing the state history with keys
#                             x, position, m, shape=(N,3)
#                             v, linear velocity, m/s, shape=(N,3)
#                             q, quaternion [i,j,k,w], shape=(N,4)
#                             w, angular velocity, rad/s, shape=(N,3)
#             t,              absolute time, s 
#         """
#         cur_state = cur_state['x']
#         # update map with new center point
#         self.local_occ_map.update(cur_state)
        
#         self.replan_num += 1
#         # Check if replanning because of potential collision

#         # TODO: you can set absolute time(t) or relative time as input, default it's absolute time
#         check_t = self.check_traj_collsion(t)
#         if self.replan_num < 20:
#             if check_t < 0 or check_t > t + 0.5 * self.traj_duration:
#                 print("No need for replanning, the check_t is : ", check_t)
#                 return
#         self.replan_num = 0

        
#         if self.exec_traj: # exec mode

#             # 1. reaching end, no plan threshld
#             if np.linalg.norm(cur_state - self.global_goal) < self.stopping_distance: 
#                 print("Reaching end ...")
#                 return
    
#             # 2. check no planning thresh
#             if np.linalg.norm(self.start - cur_state) < self.no_replan_thresh:
#                 return

#             self.exec_traj = False # transfer to replanning mode

#         else: # replanning mode
        
#             if t < self.traj_start_time + self.traj_duration: # redo planning
#                 cur_state = self.get_traj_pos(t)

#             self.start = cur_state
#             self.traj_start_time = t
#             if self.plan_traj(self.start, self.crop_local_goal(cur_state)):

#                 self.exec_traj = True
            
#         return




#     def crop_local_goal(self, start):
#         """
#         Given local start, get a straight line position as the cropped local goal 
#         and end up with the global goal
        
#         Inputs
#             start, xyz position in meters, shape=(3,)        
        
#         Outputs
#             goal,  xyz position in meters, shape=(3,)  
#         """

#         # You can also improve this function with local goal selection strategies

#         dist = np.linalg.norm(self.global_goal - start)
#         if dist <= self.planning_horizon:
#             print("reaching global goal!")
#             goal = self.global_goal
#         else:
#             # local goal here
#             goal = start + (self.planning_horizon / dist)  * (self.global_goal - start)
#         return goal

        

#     def plan_traj(self, start, goal):
#         """
#         Given local start and goal, update the trajectory

#         Inputs
#             start, xyz position in meters, shape=(3,)
#             goal,  xyz position in meters, shape=(3,)
        
#         """
#         # TODO Your code here

#         # You must store the dense path returned from your Dijkstra or AStar
#         # graph search algorithm as an object member. You will need it for
#         # debugging, it will be used when plotting results.
#         self.path = None


#         # You must generate a sparse set of waypoints to fly between. Your
#         # original Dijkstra or AStar path probably has too many points that are
#         # too close together. Store these waypoints as a class member; you will
#         # need it for debugging and it will be used when plotting results.
#         self.points = np.zeros((1,3))

#         # Finally, you must compute a trajectory through the waypoints similar
#         # to your task in the first project. One possibility is to use the
#         # WaypointTraj object you already wrote in the first project. However,
#         # you probably need to improve it using techniques we have learned this
#         # semester.

#         # You may need to update some trajectory information for replanning updates
#         self.traj_duration = 0.0



#         return True # replanning success

#     def update(self, t):
#         """
#         Given the present time, return the desired flat output and derivatives.

#         Inputs
#             t, absolute time, s 
#         Outputs
#             flat_output, a dict describing the present desired flat outputs with keys
#                 x,        position, m
#                 x_dot,    velocity, m/s
#                 x_ddot,   acceleration, m/s**2
#                 x_dddot,  jerk, m/s**3
#                 x_ddddot, snap, m/s**4
#                 yaw,      yaw angle, rad
#                 yaw_dot,  yaw rate, rad/s
#         """
#         x        = np.zeros((3,))
#         x_dot    = np.zeros((3,))
#         x_ddot   = np.zeros((3,))
#         x_dddot  = np.zeros((3,))
#         x_ddddot = np.zeros((3,))
#         yaw = 0
#         yaw_dot = 0

#         # TODO Your code here - copy your implementation in proj1.3

#         flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
#                         'yaw':yaw, 'yaw_dot':yaw_dot}
#         return flat_output




import numpy as np
from scipy.interpolate import CubicSpline

from .graph_search import graph_search
from ..util.occupancy_map import OccupancyMap
from .convert_path_to_waypoints import convert_path_to_waypoints


class WorldTraj:
    """Local‑replanning trajectory generator.

    * 不依赖任何全局路径；
    * 每次仅向最终 `global_goal` 方向延伸 `planning_horizon` 米，
      作为 **局部终点** 重新跑 A*，然后用三次样条 (CubicSpline) 拟合平滑轨迹；
    * 运行时逻辑：`replan()` —> `plan_traj()` —> 控制器持续调用
      `update(t)` 在轨迹上查询期望平坦输出；
    * 若检测到未来轨迹即将碰撞，或已飞出一定距离，则再次 `replan()`。
    """

    # ------------------------- 初始化 -------------------------
    def __init__(self, world, start, goal, local: bool = True):
        # 通用参数
        self.resolution = np.array([0.1, 0.1, 0.1])
        self.margin     = 0.3
        self.v_max      = 1                          # 规划时假设的最大速度 (m/s)

        # Local‑planning 专属参数
        self.local               = local
        self.local_occ_map       = OccupancyMap(world, self.resolution, self.margin)
        self.planning_horizon    = 4.5            # 每次局部搜索长度 (m)
        self.stopping_distance   = 0.5                 # 终点判定阈值 (m)
        self.step_t              = 0.02            # 碰撞检测采样步长 (s)
        self.no_replan_thresh    = 1.0                 # 距离上次 replanning 太近则不重规划 (m)
        self.replan_counter_max  = 20                  # 控制最低重规划频率
        self.t_check_traj        = 0.1                 # simulator 会根据此值触发 check

        # 运行时状态
        self.exec_traj      = True                    # True: 正在执行轨迹；False: 正在等待 replanning
        self.replan_counter = 0

        # 记录全局目标 & 首次规划
        self.global_goal     = np.asarray(goal,  dtype=float)
        self.start           = np.asarray(start, dtype=float)
        self.traj_start_time = 0.0

        self.local_occ_map.update(self.start)
        self.plan_traj(self.start, self._crop_local_goal(self.start))

    # --------------------------------------------------------
    # ---------------------- 关键功能函数 ---------------------
    # --------------------------------------------------------
    def _crop_local_goal(self, start: np.ndarray) -> np.ndarray:
        dist = np.linalg.norm(self.global_goal - start)
        raw_goal = self.global_goal if dist <= self.planning_horizon else start + (self.planning_horizon / dist) * (self.global_goal - start)

        # 检查raw_goal是否在障碍物里，如果是，回退一点
        if self.local_occ_map.is_occupied_metric(raw_goal):
            print("[Warning] Cropped goal inside obstacle, searching nearby...")
            step_back = 0.5 * (self.planning_horizon / dist) * (self.global_goal - start)
            raw_goal = start + step_back
        return raw_goal

    # --------------------------------------------------------
    def plan_traj(self, start: np.ndarray, goal: np.ndarray) -> bool:
        """在 *局部占据图* 上跑 A*，然后用 CubicSpline 生成平滑轨迹。"""
        # ① A* 搜索
        path, _ = graph_search(self.local_occ_map,
                               self.resolution, self.margin,
                               start, goal, astar=True)
        if path is None or len(path) < 2:
            print("[plan_traj] A* failed — no path to local goal")
            return False
        self.path = path

        # ② 稀疏化路径点（可选 — 使用 Douglas‑Peucker 简化）
        # try:
        self.points = convert_path_to_waypoints(self.path, self.resolution, self.margin)
        # except Exception:
        #     self.points = self.path.copy()

        # 保证起终点在数组首尾
        if np.linalg.norm(self.points[0] - start) > 1e-6:
            self.points = np.vstack([start, self.points])
        if np.linalg.norm(self.points[-1] - goal) > 1e-6:
            self.points = np.vstack([self.points, goal])

        # ③ 生成样条时间表（等速假设）
        seg_dists = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        seg_times = np.maximum(0.1, seg_dists / self.v_max)
        self.segment_times = np.concatenate([[0], np.cumsum(seg_times)])
        self.traj_duration = float(self.segment_times[-1])

        # ④ 构造 3D CubicSpline (clamped，首尾速度0)
        self.splines = {}
        boundary = ((1, 0.0), (1, 0.0))  # 一阶导数=0 约束
        axes = ('x', 'y', 'z')
        for i, ax in enumerate(axes):
            self.splines[ax] = CubicSpline(self.segment_times, self.points[:, i], bc_type='clamped')

        print(f"[plan_traj] new local traj: {len(self.points)} waypoints, {self.traj_duration:.2f}s duration")
        return True

    # --------------------------------------------------------
    def get_traj_pos(self, t: float) -> np.ndarray:
        """在绝对时间 *t* 上查询轨迹位置。"""
        tau = np.clip(t - self.traj_start_time, 0.0, self.traj_duration)
        return np.array([self.splines[ax](tau) for ax in ('x', 'y', 'z')])

    # --------------------------------------------------------
    def check_traj_collsion(self, cur_t: float) -> float:
        """沿当前样条前瞻，返回预测碰撞时间；若安全则返回 -1。"""
        check_t = cur_t
        while check_t < self.traj_start_time + self.traj_duration:
            check_t += self.step_t
            if self.local_occ_map.is_occupied_metric(self.get_traj_pos(check_t)):
                return check_t
        return -1.0

    # --------------------------------------------------------
    def replan(self, cur_state: dict, t: float):
        """主重规划逻辑：在 simulator 中周期性调用。"""
        pos_now = cur_state['x']
        self.local_occ_map.update(pos_now)
        self.replan_counter += 1

        # 先判断是否必须 replanning
        imminent_collision_time = self.check_traj_collsion(t)
        need_replan = (
            imminent_collision_time > 0 and imminent_collision_time <= t + 0.5 * self.traj_duration
        )
        if not need_replan and self.replan_counter < self.replan_counter_max:
            return  # 无需重规划
        self.replan_counter = 0

        # 进入 replanning
        self.start           = pos_now
        self.traj_start_time = t  # 重新计时
        local_goal           = self._crop_local_goal(pos_now)
        success = self.plan_traj(self.start, local_goal)
        self.exec_traj = success

    # --------------------------------------------------------
    # ----------------------- 控制接口 ------------------------
    def update(self, t: float) -> dict:
        """控制器每帧调用：返回期望平坦输出 (x, ẋ, ẍ, ...)。"""
        tau = np.clip(t - self.traj_start_time, 0.0, self.traj_duration)

        # 位置与导数
        x      = np.array([self.splines[ax](tau)              for ax in ('x', 'y', 'z')])
        x_dot  = np.array([self.splines[ax].derivative(1)(tau) for ax in ('x', 'y', 'z')])
        x_ddot = np.array([self.splines[ax].derivative(2)(tau) for ax in ('x', 'y', 'z')])
        x_dddot = np.array([self.splines[ax].derivative(3)(tau) for ax in ('x', 'y', 'z')])
        # CubicSpline 三阶导再微分恒为 0，因此 snap=0：
        x_ddddot = np.zeros(3)

        # yaw 设置为速度方向；若速度极小则保持上一次 yaw
        if np.linalg.norm(x_dot[:2]) > 1e-3:
            yaw = np.arctan2(x_dot[1], x_dot[0])
        else:
            yaw = 0.0
        yaw_dot = 0.0  # CubicSpline 未对 yaw 做约束，可设 0

        return {
            'x': x,
            'x_dot': x_dot,
            'x_ddot': x_ddot,
            'x_dddot': x_dddot,
            'x_ddddot': x_ddddot,
            'yaw': yaw,
            'yaw_dot': yaw_dot
        }

