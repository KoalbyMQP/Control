import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from backend.SimpleBot.RobotWF import Robot
import math
from math import cos, pi, sin
import numpy as np

if __name__ == "__main__":
    is_real = False
    robot = Robot(is_real)
    
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    max_speed = 5
    
    motor1 = robot.motors[0]
    motor2 = robot.motors[1]
    motors = [motor1, motor2]
    
    # ps1 = robot.locatePolygon()
    
    # encoders = [ps1]
    
    imu_data = robot.imu_manager.getAllIMUData()
    torso_imu = imu_data["Torso"]
    initial = torso_imu
    initial = robot.fuse_imu_data(torso_imu)
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]
    
    # for encoder in encoders:
    #     encoder.enable(timestep)
    for motor in motors:
        print("setting zero position for motor %s" % motor.name)
        motor.set_position(float('0.0'))
        motor.set_velocity(0.0)

    def set_wheels_torque(wheel1, wheel2):
        motors[0].set_torque(wheel1)
        motors[1].set_torque(wheel2)
        
    # def read_pos():
    #     return np.array([encoders[2].getValue(), encoders[5].getValue()])
    
    pose = np.array([0.0, 0.0, pi/2]) # x, y, theta
    print(pose)
    enc_last = np.array([0,0])
    radius = 0.035
    wheel_spread = 0.158
    ds = 0
    dtheta = 0
    tilt_old = 0
    k = np.array([-10, -1.2])
    k_tilt = np.array([0.08, 0.04])
    xdes = np.array([0, 0])
    pose_des = np.array([0, 0])
    pose_old = pose
    y_old = 0
    y = 0
    
sim = robot.sim
sim.setStepping(True)
timestep = sim.getSimulationTimeStep()

while True:
    sim.step()
    current_time = sim.getSimulationTime()

    imu_data = robot.imu_manager.getAllIMUData()
    torso_imu = imu_data["Torso"]

    newTargetX, newTargetY, newTargetZ = robot.IMUBalance(prevX, prevY, prevZ)
    r = math.radians(newTargetX)
    p = math.radians(newTargetY)
    y = math.radians(newTargetZ)

    tilt = r
    dtilt = tilt - tilt_old

    x = np.array([tilt, dtilt])
    e = xdes - x

    tau = np.dot(e, k)

    motor1.set_torque(tau)
    motor2.set_torque(tau)

    # enc_l = left_encoder.getValue()
    # enc_r = right_encoder.getValue()
