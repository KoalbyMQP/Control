import sys
from math import pi, atan2, degrees
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from backend.SimpleBot.RobotWF import Robot

if __name__ == "__main__":

    robot = Robot(is_real=False)

    # wheel motors
    WHEEL_R = 0
    WHEEL_L = 1
    motorL = robot.motors[WHEEL_L]
    motorR = robot.motors[WHEEL_R]

    # --- ENSURE VELOCITY MODE ---
    print("\nSetting motors to TORQUE MODE")
    for m in (motorL, motorR):
        m.enable_torque_mode_sim()

    sim = robot.sim
    sim.setStepping(True)
    dt = sim.getSimulationTimeStep()
    print(f"Using dt = {dt}s\n")

    def set_wheels_torque(tau):
        motorL.set_torque_sim(tau)
        motorR.set_torque_sim(tau)

    # Gains for velocity balancing
    Kp = 25
    Kd = 4

    max_torque = 400.0


    fall_threshold = 3

    imu_data = robot.imu_manager.getAllIMUData()
    ax, ay, az, gx, gy, gz = imu_data["Torso"]
    tilt = atan2(ax, az)    # initial pitch angle estimate (radians)


    print("Balancing with wheel TORQUE control...\n")

    try:
        while True:
            sim.step()
            t = sim.getSimulationTime()

            imu_data = robot.imu_manager.getAllIMUData()
            ax, ay, az, gx, gy, gz = imu_data["Torso"]
            # print(ax, ay, az, gx, gy, gz)

            # # IMU interpretation
            # tilt = ax       # pitch angle
            # tilt_rate = gy  # pitch rate

            alpha = 0.98     # complementary filter
            tilt_acc = atan2(ax, az)
            tilt_rate = gy
            tilt = alpha * (tilt + tilt_rate * dt) + (1 - alpha) * tilt_acc

            tilt_degrees = degrees(tilt)
            # print(tilt_degrees)
            # if abs(tilt_degrees) < fall_threshold: # fall_threshold is 5 (degrees in this context)
            #     print("Robot fell!")
            #     break
            
            # PD torque Balancing
            forward_torque = (Kp * tilt + Kd * tilt_rate)

            # clamp to max speed
            forward_torque = max(-max_torque, min(max_torque, forward_torque))
            set_wheels_torque(forward_torque)

            # print(f"t={t:.2f} | tilt={tilt:.4f} | dtilt={tilt_rate:.4f} | cmd_vel={forward_torque:.2f}")

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        try: sim.stopSimulation()
        except: pass
        try:
            if hasattr(robot, "client") and robot.client:
                robot.client.__del__()
        except: pass
