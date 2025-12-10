
# ---------------- Imports ---------------------
'''
    Import from IKPY Modules and other 
    necessary libraries (maybe OMPL for trajectory generation)
'''
import time, math, array                            # standard python packages
import pandas as pd                                 # to read in information from the gripper data file
import numpy as np                                  # to handle some of the complex math
from gripper import Gripper                         # to handle special gripper types
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
from Chess.backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from Chess.backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from Chess.backend.Testing.pickAndPlaceClass import KoalbyArmController
from Chess.backend.KoalbyHumanoid.ConfigKoalby2 import Joints

# ------------- Class Definition --------------

class PickAndPlace():
    '''
        The origin of the robot frame is the center of the base link (chest)
    '''

# -------------- Initialization ---------------

    def __init__(self, gripper, urdf_path = 'Control//Chess//backend//KoalbyHumanoid//Simulation Files//Humanoid_URDF_9-10//urdf//Humanoid_URDF_9-10.urdf', is_real = False, debug = False):
        self = self
        self.gripper = Gripper(gripper)                             # The end effector object
        self.robot = Robot2(is_real)                                # The robot object, is_real determines whether the robot is running in simulation
        self.controller = KoalbyArmController(is_real, urdf_path)   # Create controller object for arm motion
        self.debug = debug                                          # Motor debug flag              

# ----------------- Methods -------------------

    def moveToHome(self):
        '''
            Set the robot to the home position (T-Pose)
        '''

        # Set target position for all relevant motors, joint limits specified

        self.robot.motors[Joints.shoulderspin_left].target = (math.radians(0), 'P') #shoulder 1 joint - -180 to 180
        self.robot.motors[Joints.biceplift_left].target = (math.radians(0), 'P') #shoulder 2 joint - 85 sends it down to legs, -90 sends arm above head
        self.robot.motors[Joints.elbow_left].target = (math.radians(0), 'P') #Elbow joint - 110 moves towards board, may have overlap with link, -110 works
        self.robot.motors[Joints.wristspin_left].target = (math.radians(0), 'P') #Forearm joint - 180 to -180 should work
        self.robot.motors[Joints.handcurl_left].target = (math.radians(0), 'P') #Wrist joint - 90 works, but we don't need it to bend in that angle, -130 is maximum
        self.robot.motors[Joints.gripper_left].target = (math.radians(0), 'P') #nothing? will just do -180 to 180

        self.robot.motors[Joints.shoulderspin_right].target = (math.radians(0), 'P') #shoulder 1 joint - -180 to 180
        self.robot.motors[Joints.biceplift_right].target = (math.radians(0), 'P') #shoulder 2 joint - 85 sends it down to legs, -90 sends arm above head
        self.robot.motors[Joints.elbow_right].target = (math.radians(0), 'P') #Elbow joint - 110 moves towards board, may have overlap with link, -110 works
        self.robot.motors[Joints.wristspin_right].target = (math.radians(0), 'P') #Forearm joint - 180 to -180 should work
        self.robot.motors[Joints.handcurl_right].target = (math.radians(0), 'P') #Wrist joint - 90 works, but we don't need it to bend in that angle, -130 is maximum
        self.robot.motors[Joints.gripper_right].target = (math.radians(0), 'P') #nothing? will just do -180 to 180
        
        if self.debug:
            # Move motors in individually
            for m in range(12):     # only move arm motors (0 - 11)
                self.robot.moveToTarget(m)
                time.sleep(1)

        else:
            # Move all motors at once
            self.robot.moveAllToTarget()


    def getCurrentPose(self):
        '''
            Get current joint angles and coordinate position
            of joints and gripper respectively
        '''
        # If running in real world, retrieve motor positions and
        # calculate current xyz-coordinate position of end effector

        robot = self.robot
        gripper = self.gripper

        rot_axis, length, is_open = self.gripper.getGripperInfo()        
        
        # Get current motor positions
        for motor in robot.motors:
            motor.get_position()

 
    def calcTrajectory(self):
        '''
            Take an ending point and generate a
            list of waypoints to travel to that point from the current position
        '''
        # Trajectory Generation Class


    def IKToGoal(self, arm):
        '''
            Use IKPY to iterate through the waypoints in the trajectory

            Inputs:
                arm [Int]: handles which arm is making the movement (0 = left arm, 1 = right arm)
        '''

        '''
        PSEUDOCODE

        # use left arm
        if arm == 0:

            rot_axis, length, is_open = getGripperStatus()
            if not is_open:
                actuate gripper

            for waypoint in trajectory:
            
                set motor positions

                if not self.debug:
                    self.robot.moveAllToTarget()

                if self.debug:
                    using IK
                    move shoulder to location
                    move elbow to location
                    move wrist to location

        # use right arm
        elif arm == 1:

            rot_axis, length, is_open = getGripperStatus()
            if not is_open:
                actuate gripper

            for waypoint in trajectory:
            
                set motor positions

                if not self.debug:
                    self.robot.moveAllToTarget()

                if self.debug:
                    using IK
                    move shoulder to location
                    move elbow to location
                    move wrist to location

        # if neither raise exception
        else:
            raise value error, can only use 0 for left arm, 1 for right arm, no other values accepted
        '''
    
    def actuateGripper(self, arm):
        '''
            Open or close the gripper 
        '''

        # Retrieve whether the gripper is open
        _, _, is_open = self.gripper.getGripperInfo()

        if not is_open:
            if arm == 'left':
                self.robot.motors[Joints.gripper_left].target = (math.radians(45), 'P')

            elif arm == 'right':
                self.robot.motors[Joints.gripper_right].target = (math.radians(45), 'P')
        else:
            if arm == 'left':
                self.robot.motors[Joints.gripper_left].target = (math.radians(0), 'P')

            elif arm == 'right':
                self.robot.motors[Joints.gripper_right].target = (math.radians(0), 'P')

