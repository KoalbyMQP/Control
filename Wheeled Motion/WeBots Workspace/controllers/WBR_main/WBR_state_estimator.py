import numpy as np

def state_estimator(self, encoder_in, tilt_in, yaw_in):

        dwheel = (encoder_in-self.encoder_last) * self.wheel_radius
        self.encoder_last = encoder_in

        ds = np.sum(dwheel) / 2.0
        dheading = (dwheel[1]-dwheel[0]) / self.wheel_spread
        alpha = self.state_alpha
        self.velocity[0] = alpha * (ds * self.state_freq) + (1- alpha)* self.velocity[0]

        # self.velocity[1] = self.velocity[1] + dheading
        self.velocity[1] = yaw_in

        self.update_phi(tilt_in)


def find_state_space(self):
    # physical params (example values, set to your system)
        M = 2        # cart mass
        m = self.mass*3        # pendulum mass (do NOT set to 0)
        l = 0.5        # pendulum length to center of mass
        I = 2      # pendulum moment of inertia about its COM
        b = 0        # cart viscous damping (friction)
        g = 9.81       # gravity

        # common denominator used in linearized matrices
        Den = I*(M + m) + M*m*l**2

        # state order: [x, x_dot, theta, theta_dot]
        A = np.array([
            [0, 1, 0, 0],
            [0, - (I + m*l**2)*b / Den,  (m**2 * g * l**2) / Den, 0],
            [0, 0, 0, 1],
            [0, - m*l*b / Den,            m*g*l*(M + m) / Den,     0]
        ])

        # B should be a column vector (4x1)
        B = np.array([[0],
                    [(I + m*l**2) / Den],
                    [0],
                    [m*l / Den]])

        # Outputs: cart position x and pendulum angle theta
        C = np.array([[1, 0, 0, 0],
                    [0, 0, 1, 0]])

        # feedthrough (2 outputs x 1 input) — zeros
        D = np.zeros((2,1))
        return A, B, C, D