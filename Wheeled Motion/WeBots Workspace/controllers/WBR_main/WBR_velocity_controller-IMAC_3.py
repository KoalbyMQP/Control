import numpy as np
from math import cos, asin

def velocity_controller(self, vel_des):
    import numpy as np
    from math import cos, asin, pi

    # --- Linear acceleration computation ---
    k_accel = -0.6
    angle_err = vel_des[1] - self.velocity[1]  # theta_d - theta
    # dot_prod = vel_des[0] * self.velocity[0] * cos(vel_des[1] - self.velocity[1])
    # accel_des = k_accel * (self.velocity[0]**2 - dot_prod)
    accel_des = k_accel * (self.velocity[0]**2 - vel_des[0]**2) * cos(angle_err)
    if cos(angle_err) < 0:
        accel_des = -abs(accel_des)
    # v_err = vel_des[0] - self.velocity[0]
    # angle_err = vel_des[1] - self.velocity[1]
    # accel_des = k_accel * v_err * cos(angle_err)
    # accel_des = k_accel * v_err_proj

    print("accel_des is ", accel_des)
    # print("vel_des is ", vel_des[0])
    # print(self.velocity)

    # --- Angular velocity computation ---
    k_omega = 0.5
    required_turn = vel_des[1] - self.velocity[1]
    # Wrap angle to [-pi, pi]
    required_turn = (required_turn + pi) % (2 * pi) - pi
    # print("required_turn is ", required_turn)

    alpha = 3
    omega_des = k_omega * required_turn

    max_omega = 0.1 / (1 + alpha * self.velocity[0])
    new_phi_des = accel_des / 9.8
    omega_des = np.clip(omega_des, -max_omega, max_omega)
    new_phi_des = np.clip(new_phi_des, -self.max_tilt, self.max_tilt)

    # --- Wheel differential and phi_des ---
    self.wheel_differential = self.wheel_spread / self.wheel_radius * omega_des
    self.update_phi_des(new_phi_des)
    # print("new phi_des is ", self.phi_des[0])
    return