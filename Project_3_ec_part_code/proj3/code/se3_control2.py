import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2, k_F
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2 k_M
        self.gamma           = self.k_drag / self.k_thrust # k_M/k_F

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE




        # self.Kp = np.diag([33, 33, 70])     # was [20, 20, 500]]
        # self.Kd = np.diag([18, 18, 10]) * 0.6   # was [20, 20, 450]
        # self.KR = np.diag([6350.0, 6350.0, 850.0]) * 2.5 # was 23000.0, 23000.0, 13700.0]
        # self.Kw = np.diag([210.0, 210.0, 210.0]) * 2.2 # was [2750.0, 2750.0, 2300.0]
        # self.Ki = np.diag([25.0, 25.0, 33.0]) * 1.7

        # self.Kp = np.diag([40, 40, 55])     # was [20, 20, 500]]
        # self.Kd = np.diag([18, 18, 22]) * 0.7   # was [20, 20, 450]
        # self.KR = np.diag([6350.0, 6350.0, 850.0]) # was 23000.0, 23000.0, 13700.0]
        # self.Kw = np.diag([210.0, 210.0, 210.0]) * 0.6 # was [2750.0, 2750.0, 2300.0]
        # self.Ki = np.diag([25.0, 25.0, 33.0]) * 1.7

        # self.Kp = np.diag([20, 20, 30])     # was [20, 20, 500]]
        # self.Kd = np.diag([18, 18, 16]) * 0.9   # was [20, 20, 450]
        # self.KR = np.diag([6350.0, 6350.0, 850.0]) * 1.5 # was 23000.0, 23000.0, 13700.0]
        # self.Kw = np.diag([210.0, 210.0, 210.0]) * 0.8 # was [2750.0, 2750.0, 2300.0]
        # self.Ki = np.diag([25.0, 25.0, 33.0]) * 0.0



        # self.Kp = np.diag([23, 23, 26]) * 1.05    # was [20, 20, 500]]
        # self.Kd = np.diag([8, 8.5, 8.5]) * 0.55  # was [20, 20, 450]
        # self.KR = np.diag([5500.0, 5500.0, 300.0]) * 1 # was 23000.0, 23000.0, 13700.0]
        # self.Kw = np.diag([130.0, 130.0, 130.0]) *1.03 # was [2750.0, 2750.0, 2300.0]
        # self.Ki = np.diag([5.0, 5.0, 6.0]) 




        self.Kp = np.diag([22, 22, 30]) * 1.1   # was [20, 20, 500]]
        self.Kd = np.diag([8, 8.5, 6.5]) * 0.8    # was [20, 20, 450]
        self.KR = np.diag([5505.0, 5505.0, 302.0]) # was 23000.0, 23000.0, 13700.0]
        self.Kw = np.diag([120.0, 120.0, 120.0]) * 0.8 # was [2750.0, 2750.0, 2300.0]
        self.Ki = np.diag([13.0, 13.0, 13.0]) * 2.5

        self.e_int = np.zeros(3)
        self.last_t = None

    def vee(self, A):
        """
        Compute the vee operator of a 3x3 skew-symmetric matrix A.
        The matrix A should be of the form:
        [  0,   -v3,   v2 ]
        [ v3,     0,  -v1 ]
        [-v2,   v1,    0  ]
        This returns the vector [v1, v2, v3].
        """
        return np.array([A[2, 1], A[0, 2], A[1, 0]])
    
    def motor_speeds(self, thrust, moment):
        """
        This function computes the four motor speeds required to achieve a
        desired thrust and moment. The thrust is a scalar representing the
        total thrust of all four motors, and the moment is a 3D vector
        representing the total moment applied to the quadrotor.

        Inputs:
            thrust, a scalar representing the total thrust of all four motors, N
            moment, a 3D vector representing the total moment applied to the quadrotor, N*m

        Outputs:
            motor_speeds, a numpy array of length 4 with the rad/s speed of each motor
        """
        l = self.arm_length
        gamma = self.gamma
        # We need to allocate the overall force and moment commands into individual thrusts produced by each motor. 
        # We can solve this problem by solving a linear system of equations.
        # The linear system of equations is given by:
        # u1 = F1 + F2 + F3 + F4
        allocation_matrix = np.array([[1, 1, 1, 1], [0, l, 0, -l], [-l, 0, l, 0], [gamma, -gamma, gamma, -gamma]])
        # The solution to the linear system of equations is given by:
        F_i = np.linalg.solve(allocation_matrix, np.concatenate(([thrust], moment)))
        # We can now calculate the motor speeds by using the following equation:
        omega_squared = F_i / self.k_thrust
        omega_squared = np.maximum(omega_squared, 0.0)  # avoid negative values due to numerical errors
        motor_speeds = np.sqrt(omega_squared)
        return motor_speeds
        
    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        if self.last_t is None:
            dt = 0.0
        else:
            dt = t - self.last_t
            if dt < 0.0:
                dt = 0.0
        self.last_t = t

        pos_error = state['x'] - flat_output['x']    
        vel_error = state['v'] - flat_output['x_dot']
        self.e_int += pos_error * dt
        self.e_int = np.clip(self.e_int, -5.0, 5.0)
        # print(self.e_int)


        acc_target = flat_output['x_ddot'] # r_ddot_T
        # acc_desired = acc_target - self.Kd @ (state['v'] - flat_output['x_dot']) - self.Kp @ (state['x'] - flat_output['x']) # r_ddot_des
        acc_desired = (acc_target - self.Kd @ vel_error - self.Kp @ pos_error - self.Ki @ self.e_int)   # <--- integral term
        thrust_desired = self.mass * (acc_desired + np.array([0, 0, self.g])) # F_des
        current_R = Rotation.from_quat(state['q']).as_matrix() # R
        drone_z_axis = current_R[:, 2] # b3, in world frame
        cmd_thrust = np.dot(drone_z_axis, thrust_desired) # u1
        drone_z_axis_desired = thrust_desired / np.linalg.norm(thrust_desired) # b3_desired in world frame, with norm 1
        yaw_target = flat_output['yaw'] # psi_T
        heading_vector = np.array([np.cos(yaw_target), np.sin(yaw_target), 0]) # a_psi in world frame
        drone_y_axis_desired = np.cross(drone_z_axis_desired, heading_vector)/np.linalg.norm(np.cross(drone_z_axis_desired, heading_vector)) # b2_desired in world frame, with norm 1
        # print("drone_y_axis_desired", drone_y_axis_desired)
        drone_x_axis_desired = np.cross(drone_y_axis_desired, drone_z_axis_desired)
        # print("drone_x_axis_desired", drone_x_axis_desired)
        R_desired = np.column_stack((drone_x_axis_desired, drone_y_axis_desired, drone_z_axis_desired))
        # Now we need to calculate the error between R and R_desired
        error_R = 0.5 * self.vee(np.dot(R_desired.T, current_R) - np.dot(current_R.T, R_desired))
        cmd_moment = self.inertia @ (-self.KR @ error_R - self.Kw @ (state['w'] - np.zeros(3))) # u2
        # Now we need to calculate the motor speeds given the thrust and moment
        cmd_motor_speeds = self.motor_speeds(cmd_thrust, cmd_moment) # omega_i
        # Next we need to calculate the quaternion command
        cmd_q = Rotation.from_matrix(R_desired).as_quat() # q_des

        # cmd_motor_speeds = np.zeros((4,))
        # cmd_thrust = 0
        # cmd_moment = np.zeros((3,))
        # cmd_q = np.zeros((4,))

        # STUDENT CODE HERE

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input