import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid import trajPlannerPose
from backend.KoalbyHumanoid.ConfigKoalby2 import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter
from ikpy.chain import Chain

# Edit to declare if you are testing the sim or the real robot
is_real = False
robot = Robot2(is_real)
print("Setup Complete")

# --- Create URDF kinematic chains for each arm ---
# The active_links_mask tells the IK solver which joints are motorized
URDF_PATH = "backend/KoalbyHumanoid/Simulation Files/Humanoid_URDF_9-10/urdf/Humanoid_URDF_9-10.urdf"

#robot_chain = Chain.from_urdf_file(URDF_PATH, base_elements=['chest'])

left_arm_chain = Chain.from_urdf_file(URDF_PATH,
    base_elements=['shoulder_left', 'shoulderspin_left'],
    active_links_mask=[False, True, True, True, True, True, True]  # 6 active joints
)

right_arm_chain = Chain.from_urdf_file(
    URDF_PATH,
    base_elements=['shoulder_right', 'shoulderspin_right'],
    active_links_mask=[False, True, True, True, True, True, True]  # 6 active joints
)

left_leg_chain = Chain.from_urdf_file(URDF_PATH,
    base_elements=['hip_left', 'hiplift_left'],
    active_links_mask=[False, True, True, True, True, True]  # 5 active joints
)

right_leg_chain = Chain.from_urdf_file(
    URDF_PATH,
    base_elements=['hip_right', 'hiplift_right'],
    active_links_mask=[False, True, True, True, True, True]  # 5 active joints
)


def initialize():
    robot.motors[1].target = (math.radians(70), 'P')  # RightShoulderAbductor
    robot.motors[7].target = (math.radians(70), 'P') # LeftShoulderAbductor

    # Torso
    robot.motors[12].target = (math.radians(0), 'P')
    robot.motors[13].target = (math.radians(0), 'P')
    robot.motors[14].target = (math.radians(0), 'P')
    robot.motors[15].target = (0, 'P')

    # Right Leg
    robot.motors[16].target = (0, 'P')
    robot.motors[17].target = (0, 'P')
    robot.motors[18].target = (0, 'P')
    robot.motors[19].target = (0, 'P')
    robot.motors[20].target = (0, 'P')

    # Left Leg
    robot.motors[21].target = (0, 'P')
    robot.motors[22].target = (0, 'P')
    robot.motors[23].target = (0, 'P')
    robot.motors[24].target = (0, 'P')
    robot.motors[25].target = (0, 'P')


    robot.moveAllToTarget()
    print("Initial Pose Done")
    


def main():
    initialize()

    # Set initial balance targets
    imu_data = robot.imu_manager.getAllIMUData()
    right_chest_imu = imu_data["RightChest"]
    left_chest_imu = imu_data["LeftChest"]
    initial = robot.fuse_imu_data(right_chest_imu, left_chest_imu)
    prevX = initial[0]
    prevY = initial[1]
    prevZ = initial[2]

    # creates trajectory of movements (squatting knees to 80 degrees)
    simStartTime = time.time()
    #stabilizes itself before starting test
    while time.time() - simStartTime < 7:
        time.sleep(0.01)
    print("Initialized")

    while True:
        newTargetX = robot.IMUBalance(prevX, prevY, prevZ)[0]
        newTargetY = robot.IMUBalance(prevX, prevY, prevZ)[1]
        newTargetZ = robot.IMUBalance(prevX, prevY, prevZ)[2]
        print('Targets calculated')
       
        robot.motors[12].target = (math.radians(newTargetZ), 'P')  # Adjust yaw
        robot.motors[13].target = (math.radians(newTargetY), 'P')  # Adjust pitch
        robot.motors[14].target = (math.radians(-newTargetX), 'P')  # Adjust pitch

        #tells robot trajectory is specifically for arms
        robot.motors[1].target = (math.radians(70), 'P')  # RightShoulderAbductor
        robot.motors[7].target = (math.radians(70), 'P') # LeftShoulderAbductor

        # Right Leg
        robot.motors[16].target = (0, 'P')
        robot.motors[17].target = (0, 'P')
        robot.motors[18].target = (0, 'P')
        robot.motors[19].target = (0, 'P')
        robot.motors[20].target = (0, 'P')

        # Left Leg
        robot.motors[21].target = (0, 'P')
        robot.motors[22].target = (0, 'P')
        robot.motors[23].target = (0, 'P')
        robot.motors[24].target = (0, 'P')
        robot.motors[25].target = (0, 'P')

        print("Updating motors")
        robot.moveAllToTarget()
if(__name__ == "__main__"):
    main()