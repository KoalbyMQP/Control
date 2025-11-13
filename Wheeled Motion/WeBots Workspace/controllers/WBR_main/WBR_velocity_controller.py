import numpy as np
from math import cos, asin

def velocity_controller(self, vel_des):
    import numpy as np
    from math import cos, asin, pi

    # --- Linear acceleration computation ---
    k_accel = -1
    # angle_err = vel_des[1] - self.velocity[1]  # theta_d - theta
    # accel_des = k_accel * (self.velocity[0]**2 - vel_des[0]**2) * cos(angle_err)
    # if cos(angle_err) < 0:
    #     accel_des = -abs(accel_des)

    R = polar_difference(self.velocity, vel_des)
    err_proj = scalar_polar_projection(R, self.velocity)
    accel_des = k_accel * err_proj

    # if err_proj < 0 and self.velocity[0] < 0.1:
    #     accel_des = 0

    # --- Angular velocity computation ---
    k_omega = 0.5
    required_turn = vel_des[1] - self.velocity[1]
    # Wrap angle to [-pi, pi]
    required_turn = (required_turn + pi) % (2 * pi) - pi
    # print("required_turn is ", required_turn)

    alpha = 0.7
    omega_des = k_omega * required_turn

    max_omega = 0.05 / (1 + alpha * self.velocity[0])
    new_phi_des = accel_des / 9.8
    omega_des = np.clip(omega_des, -max_omega, max_omega)
    max_tilt = self.max_tilt - 0.3*abs(omega_des)
    new_phi_des = np.clip(new_phi_des, -max_tilt, max_tilt)

    # --- Wheel differential and phi_des ---
    self.wheel_differential = self.wheel_spread / self.wheel_radius * omega_des
    self.update_phi_des(new_phi_des)
    # print("new phi_des is ", self.phi_des[0])
    return

def polar_difference(p1, p2):
    """
    Compute the vector difference between two polar vectors p1 and p2.

    Parameters:
        p1, p2 : np.array([r, theta])
            r in same units, theta in radians.

    Returns:
        np.array([r_diff, theta_diff]) : the resulting vector in polar form
    """
    # Convert both to Cartesian
    x1, y1 = p1[0] * np.cos(p1[1]), p1[0] * np.sin(p1[1])
    x2, y2 = p2[0] * np.cos(p2[1]), p2[0] * np.sin(p2[1])
    
    # Subtract Cartesian components
    dx, dy = x1 - x2, y1 - y2
    
    # Convert back to polar
    r_diff = np.hypot(dx, dy)
    theta_diff = np.arctan2(dy, dx)
    
    return np.array([r_diff, theta_diff])

def scalar_polar_projection(a, b):
    return a[0] * np.cos(a[1] - b[1])