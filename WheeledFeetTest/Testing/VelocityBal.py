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
    print("\nSetting motors to VELOCITY MODE")
    for m in (motorL, motorR):
        m.set_velocity(0.0) # reset target velocity

    sim = robot.sim
    sim.setStepping(True)
    dt = sim.getSimulationTimeStep()
    print(f"Using dt = {dt}s\n")

    # Gains for velocity balancing
    Kp_vel = 150.0
    Kd_vel = 25.0
    max_speed = 400.0


    fall_threshold = 150

    imu_data = robot.imu_manager.getAllIMUData()
    ax, ay, az, gx, gy, gz = imu_data["Torso"]
    tilt = atan2(ax, az)    # initial pitch angle estimate (radians)


    print("Balancing with wheel VELOCITY control...\n")

    try:
        while True:
            sim.step()
            t = sim.getSimulationTime()

            imu_data = robot.imu_manager.getAllIMUData()
            ax, ay, az, gx, gy, gz = imu_data["Torso"]

            # # IMU interpretation
            # tilt = ax       # pitch angle
            # tilt_rate = gy  # pitch rate

            alpha = 0.98     # complementary filter
            tilt_acc = atan2(ax, az)
            tilt_rate = gy
            tilt = alpha * (tilt + tilt_rate * dt) + (1 - alpha) * tilt_acc

            tilt_degrees = degrees(tilt)
            print(tilt_degrees)
            if abs(tilt_degrees) > fall_threshold: # fall_threshold is 5 (degrees in this context)
                print("Robot fell!")
                break

            # PD Velocity Balancing
            forward_speed = (Kp_vel * tilt + Kd_vel * tilt_rate)

            # clamp to max speed
            forward_speed = max(-max_speed, min(max_speed, forward_speed))

            # send wheel velocities
            motorL.set_velocity(forward_speed)
            motorR.set_velocity(forward_speed)

           # print(f"t={t:.2f} | tilt={tilt:.4f} | dtilt={tilt_rate:.4f} | cmd_vel={forward_speed:.2f}")

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        try: sim.stopSimulation()
        except: pass
        try:
            if hasattr(robot, "client") and robot.client:
                robot.client.__del__()
        except: pass
