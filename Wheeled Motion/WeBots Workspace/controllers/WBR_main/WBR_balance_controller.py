import numpy as np
import control

def create_controller(self, M=1.0, m=0.2, l=0.5, I=0.006, b=0.1, g=9.81,
                        Q=None, R=None):
    
    # Default LQR weights
    if Q is None:
        Q = np.diag([0, 0, 100, 10])
    if R is None:
        R = np.array([[1]])

    A, B, C, D = self.find_state_space()

    # Compute LQR gain
    K, S, E = control.lqr(A, B, Q, R)

    self.K = K

    return K

# Takes in sensor data and updates self.velocity and self.phi
# TODO: MAKE THE VELOCITY OF COM NOT THE WHEELS THEMSELVES
# TODO: Add filtering to sensor inputs

def balance_controller(self):
    e = self.phi_des - self.phi
    K_pos = self.K[0, [2, 3]]   # Gains for x and theta only
    F = -np.dot(K_pos, e)
    net_torque = F * self.wheel_radius
    net_torque = np.clip(net_torque, -self.max_torque, self.max_torque)
    return net_torque