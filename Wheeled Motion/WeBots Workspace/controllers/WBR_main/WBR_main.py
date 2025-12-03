import sys
print("Webots Python path:", sys.executable)

from WBR_merge import WBR
import numpy as np
import matplotlib.pyplot as plt
from webots_interface_class import Webots
from test_rig_interface_class import TestRig

velocity_des_list = [
        np.array([1, 0]), 
        np.array([1, 3]), 
        np.array([1, -2]), 
        np.array([0, 0])
    ]

#THESE NEED TO BE EVEN FACTORS OF 1000!!!
state_freq = 200
balance_freq = 200
velocity_freq = 10

def run_state(interface, robo, velocity_des, current_time_ms):
    enc_in = interface.read_pos()
    pitch_in, yaw_in = interface.read_imu()
    robo.state_estimator(enc_in, pitch_in, yaw_in)
    return

def run_balance(interface, robo, velocity_des, current_time_ms):
    robo.balance_controller()
    interface.set_wheels_torque(robo.export_wheel_torques())
    return

def run_velocity(interface, robo, velocity_des, current_time_ms):
    robo.velocity_controller(velocity_des)
    return

loop_functions = [{"func": run_state,   "freq": 200, "next_time_ms": 0},
                  {"func": run_balance, "freq": 200, "next_time_ms": 0},
                  {"func": run_velocity,"freq": 10,  "next_time_ms": 0}]

def control_loop(interface, robo, velocity_des, current_time_ms):
    for item in loop_functions:
        if current_time_ms >= item["next_time_ms"]:                         # If overdue
            item["func"](interface, robo, velocity_des, current_time_ms)    # Run Function
            item["next_time_ms"] += 1000 / item["freq"]                     # Schedule next 
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
    robo = WBR('fred', interface.timestep, state_freq, balance_freq, velocity_freq)
    robo.create_controller()

    torque = 0
    
    while not interface.error_flag:

        interface.step()
        current_time = interface.get_time()

        velocity_desired = velocity_des_list[int(current_time // 5)]

        control_loop(interface, robo, velocity_desired, int(current_time*1000))
 
        log["phi"].append(robo.phi[0])
        log["phi_des"].append(robo.phi_des[0])
        log["time"].append(current_time)

        if current_time > 19 or abs(robo.phi[0]) > 0.2:
            print("breaking control loop")
            break

    plot_all(log)

