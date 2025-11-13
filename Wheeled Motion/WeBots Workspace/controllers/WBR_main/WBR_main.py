import sys
print("Webots Python path:", sys.executable)

from WBR_merge import WBR
from controller import Robot
from math import cos, pi, sin
import numpy as np
import matplotlib.pyplot as plt

def control_loop(robot, robo, read_pos, imu, set_wheels_torque, timestep):

    phi_store = []
    phi_des_store = []
    torque_store = []
    time_store = []
    velocity_store = []
    alpha_t_store = []
    print("starting control loop")

    torque = 0
    velocity_des_list = [np.array([0.5, 3]), np.array([1, 1]), np.array([1, -1]), np.array([1, -1])]

    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        
        # if (current_time * 1000) % 2 == 1:
        # Read sensors and estimate state
        enc_in = read_pos()
        rpy = imu.getRollPitchYaw()
        imu_in = rpy[0]
        yaw_in = rpy[2]
        robo.state_estimator(enc_in, imu_in, yaw_in)

        # Run Velocity Controller
        velocity_des = velocity_des_list[int(current_time // 5)]
        robo.velocity_controller(velocity_des)
        # print("self.phi_des is ", robo.phi_des)
        # print("self.velocity is ", robo.velocity)
        # robo.update_phi_des(0.01)

        if (current_time * 1000) % 5 == 1:
            # Run Balance Controller
            old_torque = torque
            torque = robo.balance_controller()
            alpha_max = 0.5
            k_smoothing = 5
            alpha_t = alpha_max / (1 + k_smoothing * abs(torque-old_torque))
            alpha_t = np.clip(alpha_t, 0.2, alpha_max)
            torque = alpha_t * torque  + (1.0 - alpha_t) * old_torque
            set_wheels_torque(torque + robo.wheel_differential, torque - robo.wheel_differential)

            velocity_store.append(robo.velocity[0])
            phi_store.append(imu_in)
            phi_des_store.append(robo.phi_des[0])
            torque_store.append(torque)
            time_store.append(current_time)
            alpha_t_store.append(alpha_t)

        if current_time > 15 or abs(imu_in) > 0.2:
            print("breaking control loop")
            break
    
    return phi_store, phi_des_store, torque_store, time_store, velocity_store, alpha_t_store

if __name__ == "__main__":

    robot = Robot()
    
    timestep = int(robot.getBasicTimeStep())
    # timestep = 64
    max_speed = 5
    
    motor1 = robot.getMotor('1')
    motor2 = robot.getMotor('2')
    motor3 = robot.getMotor('3')
    motor4 = robot.getMotor('4')
    motor5 = robot.getMotor('5')
    motor6 = robot.getMotor('6')
    motors = [motor1, motor2, motor3, motor4, motor5, motor6]
    
    ps1 = robot.getPositionSensor('1_sensor')
    ps2 = robot.getPositionSensor('2_sensor')
    ps3 = robot.getPositionSensor('3_sensor')
    ps4 = robot.getPositionSensor('4_sensor')
    ps5 = robot.getPositionSensor('5_sensor')
    ps6 = robot.getPositionSensor('6_sensor')
    encoders = [ps1, ps2, ps3, ps4, ps5, ps6]
    
    imu = robot.getDevice("imu")
    imu.enable(timestep)
    
    for encoder in encoders:
        encoder.enable(timestep)
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
    
    def set_hips_position(hip1, hip2):
        motors[0].setPosition(hip1)
        motors[3].setPosition(hip2)
    def set_knees_position(knee1, knee2):
        motors[1].setPosition(knee1)
        motors[4].setPosition(knee2)
    def set_wheels_torque(wheel1, wheel2):
        motors[2].setTorque(wheel1)
        motors[5].setTorque(wheel2)
        
    def read_pos():
        return np.array([encoders[2].getValue(), encoders[5].getValue()])
    
    robo = WBR('fred', timestep)
    K = robo.create_controller()
    print(K)

    phi_store, phi_des_store, torque_store, time_store, velocity_store, alpha_t_store = control_loop(robot, robo, read_pos, imu, set_wheels_torque, timestep)

    print("made it past control loop")

    phi_store = np.array(phi_store)
    phi_des_store = np.array(phi_des_store)
    torque_store = np.array(torque_store)
    time_store = np.array(time_store)
    alpha_t_store = np.array(alpha_t_store)

    print("stors converted")

    plt.figure(figsize=(10, 6))
    print("figure defined")

    plt.plot(time_store, phi_store, label='Actual Tilt')
    plt.plot(time_store, phi_des_store, label='Desired Tilt')
    plt.plot(time_store, phi_des_store-phi_store, label='Tilt Error')
    plt.plot(time_store, velocity_store, label='Velocity')
    plt.plot(time_store, alpha_t_store, label='alpha_t')
    plt.plot(time_store, torque_store, label='Torque', linestyle=':', color='r')

    print("plots variables aadded")

    plt.xlabel('Time (s)')
    plt.ylabel('Values')
    plt.title('Tilt and Torque vs Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    print("just before plotting")
    plt.show()
    print("finished past plots")