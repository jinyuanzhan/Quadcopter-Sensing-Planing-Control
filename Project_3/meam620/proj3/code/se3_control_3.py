
#  minimum jerk

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
        print('self.mass', self.mass)
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2 K_f
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2 K_m

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2
        self.gamma = self.k_drag / self.k_thrust # dimensionless
       
        
        self.structure_matrix = np.array([[1,1,1,1],
                                  [0, self.arm_length, 0, -self.arm_length],
                                  [-self.arm_length, 0, self.arm_length, 0],
                                  [self.gamma, -self.gamma, self.gamma, -self.gamma]])


        # STUDENT CODE HERE
        # PSD and diagonal matrix
        # self.k_p = np.diag([5, 5, 35])
        # self.k_d = np.diag([3.5, 4.2, 10])
        # self.k_R = np.diag([5500, 5500, 60])  
        # self.k_w = np.diag([120, 120, 120])

        # self.k_p = np.diag([6, 6, 80])
        # self.k_d = np.diag([3.5, 4.2, 20])
        # self.k_i = np.diag([0., 0., 100.0])
        # self.k_R = np.diag([5500, 5500, 70])  
        # self.k_w = np.diag([120, 120, 120])

        # self.k_p = np.diag([20, 30, 80])
        # self.k_d = np.diag([6.5, 5.2, 20])
        # # self.k_i = np.diag([25., 25., 25.0])
        # self.k_R = np.diag([5500, 5500, 300])  # the thir parameter in K_R is used to tune the yaw angle
        # self.k_w = np.diag([120, 120, 120])

        # self.k_p = np.diag([40, 40, 80])
        # self.k_d = np.diag([9.6, 9.6, 10])
        # # self.k_i = np.diag([25., 25., 25.0])
        # self.k_R = np.diag([2500, 2500, 500])  # the thir parameter in K_R is used to tune the yaw angle #like P here # roll pich yaw
        # self.k_w = np.diag([100, 100, 130]) # like D here

        # self.k_p = np.diag([8.5, 8.5, 30]) 
        # self.k_d = np.diag([5, 5, 10]) 
        # self.k_R = np.diag([1500, 1500, 80])   # attitude proportional gain
        # self.k_w = np.diag([100, 100, 20])     #  attitude derivative gain


        self.k_p = np.diag([23.5, 23.5, 83.5]) 
        self.k_d = np.diag([3, 3, 10]) 
        self.k_R = np.diag([5500, 5500, 300])  # attitude proportional gain
        self.k_w = np.diag([50, 50, 80])     #  attitude derivative gain

        # position_T = 1.77  # 1.685# desired settling time, seconds  #1.874
        # position_wn = 6.833 / position_T # natural frequency
        # self.kp_pos = (position_wn**2)*0.7
        # self.kd_pos = (2*position_wn)*0.7
        
        # if I reduce KR_1, the overshoot of i happed

        # self.integral_error = np.zeros(3)  
        # self.last_t = None                
        #KR 800


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
            flat_output target point, a dict describing the present desired flat outputs with keys
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
        # if self.last_t is None:
        #     dt = 0.0
        #     self.last_t = t
        # else:
        #     dt = t - self.last_t
        #     self.last_t = t
        #---------------------------------
        # find F_des
        x_ddot_target = flat_output['x_ddot']
        x_dot_target = flat_output['x_dot']
        x_target = flat_output['x']
        x_dot_current = state['v']
        x_current = state['x']

        # if dt > 0.0:
        #     self.integral_error += (x_current - x_target) * dt

        # x_ddot_des = x_ddot_target - self.k_d @ (x_dot_current - x_dot_target) - self.k_p @ (x_current - 
        # x_target) 

        x_ddot_des = x_ddot_target + self.k_p @ (x_target - x_current) + self.k_d @ (x_dot_target - x_dot_current)

        # - self.k_i @ self.integral_error
        F_des = (self.mass * x_ddot_des) + np.array([0,0,self.mass*self.g])
        #---------------------------------


        #---------------------------------
        #calculate u1(thrust)
        Rotation_matrix = self.quaternion_to_rotation(state['q'])
        b3 = Rotation_matrix[:,2] 
        u1 = np.dot(b3,F_des) # u1 = b3^T * F_des
        #-----------------------------------


        #-----------------------------------
        # calculate R_des and define b_i,des
        b_3_des = F_des / np.linalg.norm(F_des)
        a_phi = np.array([np.cos(flat_output['yaw']), np.sin(flat_output['yaw']), 0]) #shape (3,)
        b_2_des = np.cross(b_3_des, a_phi) / np.linalg.norm(np.cross(b_3_des, a_phi))
        b_1_des = np.cross(b_2_des, b_3_des)
        #R_des = [b_1_des, b_2_des, b_3_des]
        R_des = np.column_stack((b_1_des, b_2_des, b_3_des))
        #-----------------------------------

        #-----------------------------------
        #find e_R and e_w and calculate u2
        R_current = Rotation_matrix
        e_R = 0.5 * self.vee(R_des.T @ R_current - R_current.T @ R_des)
        w_des = np.zeros((3,)) # could be zero in this project
        e_w = state['w'] - w_des
        A =(- self.k_R @ e_R - self.k_w @ e_w)
        u2 = np.dot(self.inertia, A) #shape: (3,)
        #-----------------------------------



        # u_matrix = np.concatenate(([u1], u2))  
        # u_matrix = u_matrix.reshape((4, 1))  
        # u_matrix = np.vstack((u1, u2)) # shape 4x1
        u_matrix = np.vstack((u1.reshape(1,1), u2.reshape(3,1))) # shape 4x1
        # F_matrix = np.linalg.solve(self.structure_matrix, u_matrix) # shape 4x1
        F_matrix = np.linalg.pinv(self.structure_matrix) @ u_matrix
        omega_matrix = np.sqrt(np.maximum(F_matrix, 0) / self.k_thrust)
        for i in range(4):
            if omega_matrix[i] < self.rotor_speed_min:
                omega_matrix[i] = self.rotor_speed_min
            if omega_matrix[i] > self.rotor_speed_max:
                omega_matrix[i] = self.rotor_speed_max
        # shape (4,)
        cmd_motor_speeds = omega_matrix.flatten()
        cmd_thrust = u1
        cmd_moment = u2
        

        # cmd_motor_speeds = np.zeros((4,))
        #cmd_thrust = 0
        #cmd_moment = np.zeros((3,))
        cmd_q = self.rotation_to_quaternion(R_des)

        # STUDENT CODE HERE
        # print('cmd_motor_speeds', cmd_motor_speeds)
        # print('cmd_thrust', cmd_thrust)

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
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

    def vee(self,S):
        return np.array([S[2, 1], S[0, 2], S[1, 0]])
    
    def rotation_to_quaternion(self,R):
        """
        Convert a rotation matrix to a quaternion.

        Inputs:
            R, rotation matrix

        Outputs:
            q, quaternion [i,j,k,w]
        """
        w = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
        return np.array([x, y, z, w])
    









