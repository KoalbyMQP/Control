import time
import numpy as np
from RobotKoalby2 import Robot2
import modern_robotics as mr
from trajPlannerTime import TrajPlannerTime

#task space planner, originally designed for end effector alignment with the swapping station

class TaskPlanner ():
    #robot: the current robot object of type RobotKoalby2
    #finalPos: a XYZ vector for the final position of the end effector
    #finalRot: rotation matrix of the final position of the end effector
    #whichArm: string of the acceptable left or right forms
    #interval: interval of time at which the motors make an attempt to reach the current frame along the trajectory

    def __init__(self, robot: Robot2, finalPos, finalRot,whichArm, interval= 0.1): #all array-like parameters are assumed to be numpy objects
        self.robot = robot
        self.arm = self.whichArm(whichArm) #"Left", "left" or L
        self.interval = interval #resolution at which the trajectories are ran

        self.recalcTime = 0.5 #time interval to recalculate the trajectory

        self.targetAngles = np.zeros(6)#6 motors long for all motors in the arm

        self.finalT = mr.RpToTrans(finalRot, finalPos)

        self.startT = self.updateCurrentPose()

        self.CURRENT_IDS = self.setCurrentMotorIDs()


        # Order: shoulderspin, biceplift, elbow, wristspin, handcurl, gripper
        self.LEFT_ARM_MOTOR_IDS = [7,3,2,1,26,33
            #ID_for_shoulderspin_left,
            #ID_for_biceplift_left,
            #ID_for_elbow_left,
            #ID_for_wristspin_left,
            #ID_for_handcurl_left,
            #ID_for_gripper_left
        ]

        # Order: shoulderspin, biceplift, elbow, wristspin, handcurl, gripper
        self.RIGHT_ARM_MOTOR_IDS = [15,6,10,11,25,32
            #ID_for_shoulderspin_right,
            #ID_for_biceplift_right,
            #ID_for_elbow_right,
            #ID_for_wristspin_right,
            #ID_for_handcurl_right,
            #ID_for_gripper_right
        ]
    
    def setCurrentMotorIDs(self):
        if self.arm == "right_gripper":
            return self.RIGHT_ARM_MOTOR_IDS
        else:
            return self.LEFT_ARM_MOTOR_IDS

    def whichArm(self, parameter):#arm is denoted by the gripper motor tag
        if (parameter in ["Right", "right", "R"]):
            return "right_gripper"
        elif(parameter in ['Left', 'left', 'L']):
            return "left_gripper"
        
    def updateTargetPose(self,T):
        self.finalT = T

    def updateCurrentPose(self):
        return self.robot.locate(self.arm)
        
    def runTrajectory(self, time):
        startTime = time.time() # time at trajectory start

        thetaList = np.array()
        next = self.robot.chain[self.arm]
        while next != "base": #grab current location to use as best 
            thetaList.append(next.get_position())
            next = self.chain[next.name]

        thetaList.reverse()

        currTraj = TrajPlannerTime([0, time], [self.startT.flatten(), self.finalT.flatten()], [0,0], [0,0])#run trajectories on every number in homogenous tranform, calculated transforms might not contain proper SO(3) rotation matrices due to interpolation
        while time.time() < startTime + time: #outer loop where every half second the trajectory is recalculated
            while (time.time() - startTime)%0.5 > 0.499: #inner loop where trajectory is followed
                loopTime = time.time()
                flatT = currTraj.getQuinticPositions() #is not angle positions, is a flattened, interpolated transform matrix
                T = np.reshape(flatT, [4,4]) #reshapes into 4 by 4
                targetAngs = self.robot.IK(self.arm, T, thetaList)

                for i, motor_id in enumerate(self.CURRENT_IDS):
                    angle = targetAngs[i + 1]  # +1 to skip the non-motorized base link
                    motor_object = self.robot.getMotor(motor_id) # Find the motor in the list
                if motor_object:
                    motor_object.target = (angle, 'P')
                else:
                    print(f"Warning: Left arm motor with ID {motor_id} not found.")
                
                while(time.time() - loopTime) < self.interval:
                    time.sleep(0.0001)
                

            currTraj.recalc([self.startT.flatten(), self.finalT.flatten()])

            
        