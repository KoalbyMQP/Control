from controller import Robot
import numpy as np

class Webots:
    def __init__(self):
        self.robot = Robot()
    
        self.timestep = int(self.robot.getBasicTimeStep())
        print(self.timestep)
        
        motor1 = self.robot.getMotor('1')
        motor2 = self.robot.getMotor('2')
        motor3 = self.robot.getMotor('3')
        motor4 = self.robot.getMotor('4')
        motor5 = self.robot.getMotor('5')
        motor6 = self.robot.getMotor('6')
        self.motors = [motor1, motor2, motor3, motor4, motor5, motor6]
        
        ps1 = self.robot.getPositionSensor('1_sensor')
        ps2 = self.robot.getPositionSensor('2_sensor')
        ps3 = self.robot.getPositionSensor('3_sensor')
        ps4 = self.robot.getPositionSensor('4_sensor')
        ps5 = self.robot.getPositionSensor('5_sensor')
        ps6 = self.robot.getPositionSensor('6_sensor')
        self.encoders = [ps1, ps2, ps3, ps4, ps5, ps6]
        
        self.imu = self.robot.getDevice("imu")
        self.imu.enable(self.timestep)
        
        for encoder in self.encoders:
            encoder.enable(self.timestep)
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
    # def set_hips_position(self, hip1, hip2):
    #     self.motors[0].setPosition(hip1)
    #     self.motors[3].setPosition(hip2)
    # def set_knees_position(self, knee1, knee2):
    #     self.motors[1].setPosition(knee1)
    #     self.motors[4].setPosition(knee2)
    
    def set_wheels_torque(self, torques):
        self.motors[2].setTorque(torques[0])
        self.motors[5].setTorque(torques[1])
        
    def read_pos(self):
        return np.array([self.encoders[2].getValue(), self.encoders[5].getValue()])
    
    # Moves 1 timestep in the simulation
    def step(self):
        return self.robot.step(self.timestep)
    
    # Returns time in seconds
    def get_time(self):
        return self.robot.getTime()
    
    # Returns pitch, yaw
    def read_imu(self):
        rpy = self.imu.getRollPitchYaw()
        return rpy[0], rpy[2]