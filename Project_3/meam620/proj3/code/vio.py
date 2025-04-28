
import numpy as np
from numpy.linalg import inv, norm
from scipy.spatial.transform import Rotation
def skew(vec):
    x, y, z = vec
    return np.array([
        [0, -z,  y],
        [z,  0, -x],
        [-y, x,  0]
    ])


def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    Perform the nominal state update based on IMU measurements (angular velocity w_m, linear
    acceleration a_m) over a time interval dt.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                         p, v, a_b, w_b, g are 3x1 column vectors,
                         q is a scipy.spatial.transform.Rotation object
    :param w_m: 3x1 measured angular velocity (rad/s)
    :param a_m: 3x1 measured linear acceleration (m/s^2)
    :param dt:  scalar time step (s)
    :return: new tuple (p_{k+1}, v_{k+1}, q_{k+1}, a_b, w_b, g)
    """
    p, v, q, a_b, w_b, g = nominal_state

    w = (w_m - w_b).ravel()  # shape (3,) angular velocity without bias
    a = (a_m - a_b).ravel()  # shape (3,) 
    R_k = q.as_matrix()  # 3x3
    new_p = p + v * dt + 0.5 * (R_k @ a.reshape(3,1) + g) * (dt**2)
    new_v = v + (R_k @ a.reshape(3,1) + g) * dt
    delta_q = Rotation.from_rotvec(w * dt)  # delta q
    new_q = (q * delta_q) # quaternion multiplication
    return new_p, new_v, new_q, a_b, w_b, g

def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """
    p, v, q, a_b, w_b, g = nominal_state

    w = (w_m - w_b).ravel()
    a = (a_m - a_b).ravel()
    R_k = q.as_matrix()


    
    F = np.zeros((18,18))
    F[0:3, 0:3] = np.eye(3)
    F[0:3, 3:6] = np.eye(3) * dt

    F[3:6, 3:6] = np.eye(3)
    F[3:6, 6:9] = -R_k @ skew(a) * dt
    F[3:6, 9:12] = -R_k * dt
    F[3:6,15:18] = np.eye(3) * dt
    # r= Rotation.from_rotvec((w * dt).ravel())
    # r = r.as_matrix()
    F[6:9, 6:9] = Rotation.from_rotvec((w * dt).ravel()).as_matrix().T
    F[6:9, 12:15] = -np.eye(3) * dt
    
    F[9:12, 9:12] = np.eye(3)
    F[12:15, 12:15] = np.eye(3)
    F[15:18, 15:18] = np.eye(3)




    G = np.zeros((18, 12))
    G[6:9, 3:6] = np.eye(3)
    G[3:6, 0:3] = np.eye(3)
    G[9:12, 6:9] = np.eye(3)
    G[12:15, 9:12] = np.eye(3)
    
    Qd = np.zeros((12, 12))
    Qd[0:3, 0:3] = accelerometer_noise_density**2 * (dt**2) * np.eye(3)
    Qd[3:6, 3:6] = gyroscope_noise_density**2     * (dt**2) * np.eye(3)
    Qd[6:9, 6:9] = accelerometer_random_walk**2    * (dt)      * np.eye(3)
    Qd[9:12, 9:12] = gyroscope_random_walk**2       * (dt)     * np.eye(3)
    new_error_covariance = F @ error_state_covariance @ F.T + G @ Qd @ G.T
    return new_error_covariance


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
   
    p, v, q, a_b, w_b, g = nominal_state
    r = q.as_matrix()
    Pc = r.T @ ((Pw - p).ravel())
    Xc, Yc, Zc = Pc

    z_pred = np.array([[Xc / Zc], [Yc / Zc]])  # 2x1

    innovation = uv - z_pred
    if norm(innovation) >= error_threshold:
        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
    J_Pc = np.array([
        [1/Zc,     0.0,         -Xc/(Zc*Zc)],
        [0,        1/Zc,      -Yc/(Zc*Zc)]
    ])
    R_inv = q.inv().as_matrix()

    dPc_dp = -R_inv
    dPc_dtheta = skew(Pc)
    H = np.zeros((2, 18))
    H[0:2, 0:3] = J_Pc @ dPc_dp
    H[0:2, 6:9] = J_Pc @ dPc_dtheta
    S = H @ error_state_covariance @ H.T + Q  # 2x2
    K = error_state_covariance @ H.T @ inv(S) # (18x2)
    delta_x = K @ innovation  # 18x1

    delta_p  = delta_x[0:3]   # 0~2
    delta_v  = delta_x[3:6]   # 3~5
    delta_th = delta_x[6:9]   # 6~8
    delta_ab = delta_x[9:12]  # 9~11
    delta_wb = delta_x[12:15] # 12~14
    delta_g  = delta_x[15:18] # 

    new_p  = p  + delta_p.reshape(3,1)
    new_v  = v  + delta_v.reshape(3,1)
    new_ab = a_b + delta_ab.reshape(3,1)
    new_wb = w_b + delta_wb.reshape(3,1)
    new_g  = g  + delta_g.reshape(3,1)

    delta_q = Rotation.from_rotvec(delta_th.ravel())
    new_q   = (q * delta_q)
    I18 = np.eye(18)
    KH  = K @ H
    new_error_covariance = (I18 - KH) @ error_state_covariance @ (I18 - KH).T + K @ Q @ K.T
    new_state = (new_p, new_v, new_q, new_ab, new_wb, new_g)
    return new_state, new_error_covariance, innovation
