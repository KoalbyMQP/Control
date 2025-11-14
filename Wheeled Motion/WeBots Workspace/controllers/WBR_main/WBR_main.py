import sys
print("Webots Python path:", sys.executable)

from WBR_merge import WBR
import numpy as np
import matplotlib.pyplot as plt
from webots_interface_class import Webots

def control_loop(interface, robo, velocity_des, current_time):

    enc_in = interface.read_pos()
    rpy = interface.read_imu()

    robo.state_estimator(enc_in, rpy[0], rpy[2])

    robo.velocity_controller(velocity_des)

    # run balance controller occasionally
    if (current_time * 1000) % 5 == 1:
        torque = robo.balance_controller()
        robo.update_torque(torque)
        interface.set_wheels_torque(robo.export_wheel_torques())

    return
 
def plot_all(logs):
    """
    Plot all logged signals from the control loop.

    logs: dict returned by control_loop()
          Expected keys: 'time', 'phi', 'phi_des', 'torque', 'velocity', 'alpha_t'
    """
    # Convert all logs to numpy arrays for easier plotting
    time_store = np.array(logs['time'])
    phi_store = np.array(logs['phi'])
    phi_des_store = np.array(logs['phi_des'])

    plt.figure(figsize=(10, 6))

    plt.plot(time_store, phi_store, label='Actual Tilt')
    plt.plot(time_store, phi_des_store, label='Desired Tilt')

    plt.xlabel('Time (s)')
    plt.ylabel('Values')
    plt.title('Tilt and Torque vs Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":

    log = {
        "phi": [],
        "phi_des": [],
        "time": []
    }

    interface = Webots()
    robo = WBR('fred', interface.timestep)
    robo.create_controller()

    torque = 0
    velocity_des_list = [
        np.array([0.5, 3]), 
        np.array([1, 1]), 
        np.array([1, -1]), 
        np.array([1, -1])
    ]
    

    while interface.step() != -1:
        current_time = interface.get_time()

        velocity_des = velocity_des_list[int(current_time // 5)]

        control_loop(interface, robo, velocity_des, current_time)
 
        log["phi"].append(robo.phi[0])
        log["phi_des"].append(robo.phi_des[0])
        log["time"].append(current_time)

        if current_time > 15 or abs(robo.phi[0]) > 0.2:
            print("breaking control loop")
            break

    plot_all(log)

