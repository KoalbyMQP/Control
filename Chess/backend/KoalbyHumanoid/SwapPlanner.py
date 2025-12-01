import sys
import time
import numpy as np
import modern_robotics as mr
import scipy.spacial.transform.Rotation as Rotation

#sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime

#task space planner, originally designed for end effector alignment with the swapping station

class SwapPlanner ():
    #robot: the current robot object of type RobotKoalby2
    #finalPos: a XYZ vector for the final position of the end effector
    #finalRot: rotation matrix of the final position of the end effector
    #whichArm: string of the acceptable left or right forms
    #interval: interval of time at which the motors make an attempt to reach the current frame along the trajectory

    def __init__(self, robot: Robot2, finalPos, finalRot, trajTime, whichArm, interval= 0.1): #all array-like parameters are assumed to be numpy arrays objects
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
        
        self.robot = robot
        self.arm = self.whichArm(whichArm) #"Left", "left" or L
        self.interval = interval #resolution at which the trajectories are ran

        self.trajTime = trajTime #total time to run the trajectory

        self.recalcTime = 0.5 #time interval to recalculate the trajectory

        self.targetAngles = np.zeros(6)#6 motors long for all motors in the arm

        self.finalT = mr.RpToTrans(finalRot, finalPos)

        self.CURRENT_IDS = self.setCurrentMotorIDs()

        self.startT = self.updateCurrentPose()

        self.coeffs = self.calcTrajectory(self.startT, self.finalT)#matrix of 6 rows of coeffs for each of the 6 control dimensions (x,y,z,phi,theta,psi)

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
            return "gripper_right"
        elif(parameter in ['Left', 'left', 'L']):
            return "gripper_left"
        
    def updateTargetPose(self,T):
        self.finalT = T

    def updateCurrentPose(self):
        #print(self.arm)
        return self.robot.locate(self.robot.getMotor(self.CURRENT_IDS[5]))
        
    def runTrajectory(self):
        startTime = time.time() # time at trajectory start

        thetaList = np.array([])
        next = self.robot.chain[self.arm]
        while next != "base": #grab current location to use as best 
            thetaList.append(next.get_position())
            next = self.chain[next.name]

        np.flip(thetaList) #reverse to proper order from base to end effector

        #currTraj = TrajPlannerTime([np.zeros((self.startT.flatten().size)), _time * (np.ones((self.startT.flatten().size))) ], [self.startT.flatten(), self.finalT.flatten()], [0,0], [0,0])#run trajectories on every number in homogenous tranform, calculated transforms might not contain proper SO(3) rotation matrices due to interpolation
        while time.time() < startTime + self.trajTime: #outer loop where every half second the trajectory is recalculated
            while ((time.time() - startTime)%0.5 > 0.499) and (time.time() < startTime + self.trajTime): #inner loop where trajectory is followed
                loopTime = time.time()
                #flatT = currTraj.getQuinticPositions() #is not angle positions, is a flattened, interpolated transform matrix
                #T = np.reshape(flatT, [4,4]) #reshapes into 4 by 4
                targetAngs = self.calcTargetAngs(time.time() - startTime, thetaList)

                for i, motor_id in enumerate(self.CURRENT_IDS):
                    angle = targetAngs[i + 1]  # +1 to skip the non-motorized base link
                    motor_object = self.robot.getMotor(motor_id) # Find the motor in the list
                if motor_object:
                    motor_object.target = (angle, 'P')
                else:
                    print(f"Warning: Left arm motor with ID {motor_id} not found.")
                
                self.robot.moveAllToTarget()

                while(time.time() - loopTime) < self.interval:
                    time.sleep(0.0001)
                

            self.recalcTrajectory()#update trajectory from stored poses

    def recalcTrajectory(self):
        #self.startT = self.updateCurrentPose()
        self.coeffs = self.calcTrajectory(self.startT, self.finalT)
    
    def calcTrajectory(self, startT, finalT):
        startRot = Rotation.from_matrix(startT[0:2,0:2])
        finalRot = Rotation.from_matrix(finalT[0:2,0:2])
        startPos = startT[0:2,3]
        finalPos = finalT[0:2,3]
        
        xConds = np.array([startPos[0], 0, 0, finalPos[0], 0, 0])#vectors of conditions for matrix solving
        yConds = np.array([startPos[1], 0, 0, finalPos[1], 0, 0])
        zConds = np.array([startPos[2], 0, 0, finalPos[2], 0, 0])
        pConds = np.array([startRot.as_euler('xyz')[0], 0, 0, finalRot.as_euler('xyz')[0], 0, 0])#phi - roll - along x
        tConds = np.array([startRot.as_euler('xyz')[1], 0, 0, finalRot.as_euler('xyz')[1], 0, 0])#theta - pitch - along y
        sConds = np.array([startRot.as_euler('xyz')[2], 0, 0, finalRot.as_euler('xyz')[2], 0, 0])#psi - yaw - along z

        bigM = np.stack((self.Mt(0), self.Mt(self.trajTime)))

        xCoeffs = np.linalg.lstsq(bigM, xConds)[0]
        yCoeffs = np.linalg.lstsq(bigM, yConds)[0]
        zCoeffs = np.linalg.lstsq(bigM, zConds)[0]
        pCoeffs = np.linalg.lstsq(bigM, pConds)[0]
        tCoeffs = np.linalg.lstsq(bigM, tConds)[0]
        sCoeffs = np.linalg.lstsq(bigM, sConds)[0]
        return np.array([xCoeffs], [yCoeffs], [zCoeffs], [pCoeffs], [tCoeffs], [sCoeffs])
    
    def calcTargetAngs(self, currentTime, angGuess):
        x = np.dot(self.Mt(currentTime), self.coeffs[0])
        y = np.dot(self.Mt(currentTime), self.coeffs[1])
        z = np.dot(self.Mt(currentTime), self.coeffs[2])
        phi = np.dot(self.Mt(currentTime), self.coeffs[3])
        theta = np.dot(self.Mt(currentTime), self.coeffs[4])
        psi = np.dot(self.Mt(currentTime), self.coeffs[5])

        rot = Rotation.from_euler('xyz', [phi, theta, psi]).as_matrix()
        T = mr.RpToTrans(rot.as_matrix(), [x, y, z])
        targetAngs = self.robot.IK(self.arm, T, angGuess)
        return targetAngs

        

    
    def Mt(self, t):
        return np.array([
            [1, t, t**2, t**3, t**4, t**5],
            [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],
            [0, 0, 2, 6*t, 12*t**2, 20*t**3]
        ])
