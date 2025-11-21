import numpy as np

class WBR:
    def __init__(self, name, timestep, state_freq, balance_freq, velocity_freq):
        self.name = name
        self.q = np.array([0, 0, 0])            # (hip, knee, wheel)
        self.position = np.array([0, 0, 0])     # (x, y, theta)
        self.encoder_last = np.array([0, 0])
        self.velocity = np.array([0.0, 0.0])        # (speed, heading)
        self.phi = np.array([0.0, 0.0])            # (phi, dphi)
        self.phi_des = np.array([0.0, 0.0])
        self.timestep_s = timestep / 1000.0
        self.mass = 10
        self.wheel_radius = 0.035
        self.wheel_spread = 0.158
        self.max_torque = 0.3
        self.max_tilt = 0.05
        self.wheel_differential = 0
        self.torque = 0
        self.state_freq = state_freq
        self.balance_freq = balance_freq
        self.velocity_freq = velocity_freq
        self.state_alpha = 1 - np.exp(-2 * np.pi * 1 * (1/state_freq)) # 1 Hz cutoff freq

    # Updates phi and dphi
    def update_phi(self, new_phi):
        self.phi[1] = (new_phi - self.phi[0]) * self.state_freq
        self.phi[0] = new_phi

    def update_phi_des(self, new_phi):
        self.phi_des[1] = (new_phi - self.phi_des[0]) * self.velocity_freq
        self.phi_des[0] = new_phi

    def export_wheel_torques(self):
        return np.array([self.torque + self.wheel_differential,
                         self.torque - self.wheel_differential])