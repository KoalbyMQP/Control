from controller import Robot
from math import cos, pi, sin
import numpy as np

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
    
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        
        vy = (y - y_old) * 1000.0/timestep
        e_y = np.array([pose_des[0] - pose[1], pose_des[1] - vy])
        print("e_y is %f, %f " % (e_y[0], e_y[1]))
        tilt_des = np.dot(k_tilt, e_y)
        xdes = np.array([tilt_des, 0])
        print("dy is %f and tilt_des %f" % (e_y[1], tilt_des))
        
        rpy = imu.getRollPitchYaw()
        tilt = rpy[0]
        dtilt = tilt - tilt_old
        
        x = np.array([tilt, dtilt])
        e = xdes - x
        
        if tilt_des < 0.1:
            tau = np.dot(e, k)
            # print("applying torque %f" % tau)
            set_wheels_torque(tau, tau)
        
        enc = read_pos()
        dx = (enc-enc_last) * radius
        ds = np.sum(dx) / 2.0
        dtheta = (dx[1]-dx[0]) / wheel_spread
        
        y_old = pose[1]
        pose[0] = pose[0] + ds*cos(pose[2])
        pose[1] = pose[1] + ds*sin(pose[2])
        pose[2] = pose[2] + dtheta
        y = pose[1]
        
        # print("x, y, theta is")
        # print(pose)
        # print("Linear Velocities are %f, %f" % (dx[0], dx[1]))
        # print("ds, dtheta are %f, %f" % (ds, dtheta))
        enc_last = enc
        