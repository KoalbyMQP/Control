
# ---------------- Imports ---------------------
'''
    Import from IKPY Modules and other 
    necessary libraries (maybe OMPL for trajectory generation)
'''
import pandas as pd                                                         # to read in information from the gripper data file
import numpy as np                                                          # to handle some of the complex math
from gripper import Gripper                                                 # to get gripper info for IK and motion planning
from Chess.backend.KoalbyHumanoid.Robot import Robot                        # to create robot urdf model
from Chess.backend.KoalbyHumanoid.trajPlannerPose import TrajPlannerPose    # to handle trajectory planning

# ------------- Class Definition --------------

class PickAndPlace():
    '''
        The origin of the robot frame is the center of the base link (chest)
    '''
    
    def __init__(self, gripper, urdf_path, is_real = False, debug = False):
        self = self
        self.gripper = Gripper(gripper)                 # The end effector object
        self.robot_model = Robot(is_real = is_real)     # The robot object, is_real determines whether the robot is running in simulation
        self.debug = debug                              # Motor debug flag              

    def moveToHome(self):
        '''
            Set the robot to the home position (T-Pose)
        '''
        # Use calcTrajectory and inverseKinematics to set joints to a T-Pose
    
    def getCurrentPose(self):
        '''
            Get current joint angles and coordinate position
            of joints and gripper respectively
        '''
        # Read joints, from IK
 
    def calcTrajectory(self):
        '''
            Take a starting point and ending point and generate a
            list of waypoints to travel to
        '''

        # Trajectory Generation Class

    def inverseKinematics(self):
        '''
            Use IKPY to iterate through the waypoints in the trajectory
        '''

        '''
        PSEUDOCODE

        rot_axis, length, is_open = getGripperStatus()
        if not is_open:
            actuate gripper

        for waypoint in trajectory
            if not debug:
                chain arm
                use robot IK to move to way point

            if debug:
                using IK
                move shoulder to location
                move elbow to location
                move wrist to location
        '''
    
    def actuateGripper(self):
        '''
            Open or close the gripper
        '''

        '''
        rot_axis, length, is_open = getGripperStatus()
        if not is_open:
            actuate gripper to open
        else:
            actuate gripper to closed
        '''