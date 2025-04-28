import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):

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
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2
        self.gamma = self.k_drag / self.k_thrust # dimensionless
        self.structure_matrix = np.array([
            [1, 1, 1, 1],
            [0, self.arm_length, 0, -self.arm_length],
            [-self.arm_length, 0, self.arm_length, 0],
            [self.gamma, -self.gamma, self.gamma, -self.gamma]
        ])
        
       
        # self.k_p = np.diag([8, 8, 80])  # 位置比例增益
        # self.k_d = np.diag([5.5, 5.25, 50])   
        # self.k_i = np.diag([3.5, 3.5, 3.5])  
        # self.k_R = np.diag([5500, 5500, 500]) 
        # self.k_w = np.diag([120, 120, 120])  # 
        self.k_p = np.diag([23, 23, 26] )* 1.05    # position proportional gain
        self.k_d = np.diag([8,8.5, 8.5]) * 0.55    # acceleration differential gain
        self.k_i = np.diag([5, 5, 6])               # integral gain
        self.k_R = np.diag([5500, 5500,300])* 1     #  oreintation proportional gain 
        self.k_w = np.diag([130, 130, 130])* 1.03   # angular velocity differential gain

        # self.k_p = np.diag([8.5, 8.5, 30])
        # self.k_d = np.diag([5, 5, 10])
        # self.k_R = np.diag([1500, 1500, 80])  # attitude proportional gain
        # self.k_w = np.diag([100, 100, 20]) #  attitude derivative gain
        
        self.integral_error = np.zeros(3)
        self.max_integral = np.array([2.0, 2.0, 2.0])  
        self.prev_error = np.zeros(3) 
        self.last_t = None

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
        self.last_t = t
        x_target = flat_output['x']
        x_dot_target = flat_output['x_dot']
        x_ddot_target = flat_output['x_ddot']
        x_current = state['x']
        x_dot_current = state['v']
        
       
        position_error = x_target - x_current
        velocity_error = x_dot_target - x_dot_current
        if dt > 0.0:
            
            if np.linalg.norm(position_error) < 0.05:

                self.integral_error = np.zeros(3)
            elif np.any(position_error * self.prev_error < 0):
                self.integral_error *= 0.5
            self.integral_error += position_error * dt
            self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        self.prev_error = position_error.copy()

        x_ddot_des = x_ddot_target + self.k_d @ velocity_error + self.k_p @ position_error + self.k_i @ self.integral_error
        

        F_des = self.mass * x_ddot_des + np.array([0, 0, self.mass * self.g])
        

        R_current = self.quaternion_to_rotation(state['q'])
        b3 = R_current[:, 2]  
        u1 = np.dot(b3, F_des)  
        

        b3_des = F_des / np.linalg.norm(F_des)  

        a_phi = np.array([np.cos(flat_output['yaw']), np.sin(flat_output['yaw']), 0])
        b2_des = np.cross(b3_des, a_phi)
        b2_des = b2_des / np.linalg.norm(b2_des) if np.linalg.norm(b2_des) > 1e-6 else np.array([0, 1, 0])
        b1_des = np.cross(b2_des, b3_des)
        R_des = np.column_stack((b1_des, b2_des, b3_des))
        
 
        e_R = 0.5 * self.vee(R_des.T @ R_current - R_current.T @ R_des)
        

        w_des = np.zeros(3)
        if abs(flat_output['yaw_dot']) > 1e-6:

            w_des[2] = flat_output['yaw_dot'] 
        

        e_w = state['w'] - w_des
        
   
        u2 = self.inertia @ (- self.k_R @ e_R - self.k_w @ e_w)
        
 
        u_matrix = np.vstack((u1.reshape(1, 1), u2.reshape(3, 1)))  # 4x1 matrix [F, M_x, M_y, M_z]^T
        F_matrix = np.linalg.pinv(self.structure_matrix) @ u_matrix
        
 
        F_matrix = np.maximum(F_matrix, 0)
        omega_matrix = np.sqrt(F_matrix / self.k_thrust)
        

        cmd_motor_speeds = np.clip(omega_matrix.flatten(), self.rotor_speed_min, self.rotor_speed_max)
        

        cmd_thrust = u1
        cmd_moment = u2
        cmd_q = self.rotation_to_quaternion(R_des)
        
        control_input = {
            'cmd_motor_speeds': cmd_motor_speeds,
            'cmd_thrust': cmd_thrust,
            'cmd_moment': cmd_moment,
            'cmd_q': cmd_q
        }
        
        return control_input
    
    def quaternion_to_rotation(self, q):
        """
        Convert a quaternion to a rotation matrix.

        Inputs:
            q, quaternion [i,j,k,w]

        Outputs:
            R, rotation matrix
        """
        i, j, k, w = q
        R = np.array([
            [1 - 2*j**2 - 2*k**2, 2*i*j - 2*w*k, 2*i*k + 2*w*j],
            [2*i*j + 2*w*k, 1 - 2*i**2 - 2*k**2, 2*j*k - 2*w*i],
            [2*i*k - 2*w*j, 2*j*k + 2*w*i, 1 - 2*i**2 - 2*j**2]
        ])
        return R

    def vee(self, S):
        return np.array([S[2, 1], S[0, 2], S[1, 0]])
    
    def rotation_to_quaternion(self, R):
        """
        Convert a rotation matrix to a quaternion.

        Inputs:
            R, rotation matrix

        Outputs:
            q, quaternion [i,j,k,w]
        """
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
            
        return np.array([x, y, z, w])