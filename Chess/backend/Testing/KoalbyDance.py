#Notes: This code is under dev and is not final.

# ============================================================================
# IMPORTS AND INITIAL SETUP
# ============================================================================
import sys
import time
import math
import numpy as np
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils

sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime

# --- Create URDF kinematic chains for each arm ---
# The active_links_mask tells the IK solver which joints are motorized
URDF_PATH = "backend/KoalbyHumanoid/Simulation Files/Humanoid_URDF_9-10/urdf/Humanoid_URDF_9-10.urdf"

left_arm_chain = Chain.from_urdf_file(URDF_PATH,
    base_elements=['shoulder_left', 'shoulderspin_left'],
    active_links_mask=[False, True, True, True, True, True, True]  # 6 active joints
)

right_arm_chain = Chain.from_urdf_file(
    URDF_PATH,
    base_elements=['shoulder_right', 'shoulderspin_right'],
    active_links_mask=[False, True, True, True, True, True, True]  # 6 active joints
)

# --- Connect to the robot (or simulation) ---
is_real = False
robot = Robot2(is_real)
print("Setup Complete")


# ============================================================================
# MOTOR CONFIGURATION AND INITIALIZATION
# ============================================================================

# --- Define Motor IDs for each arm ---
# The order MUST match the joint order from the URDF file analysis.
# Replace the placeholder comments with the real IDs you find from the wiggle test.

# Order: shoulderspin, biceplift, elbow, wristspin, handcurl, gripper
LEFT_ARM_MOTOR_IDS = [7,3,2,1,26,33
    #ID_for_shoulderspin_left,
    #ID_for_biceplift_left,
    #ID_for_elbow_left,
    #ID_for_wristspin_left,
    #ID_for_handcurl_left,
    #ID_for_gripper_left
]

# Order: shoulderspin, biceplift, elbow, wristspin, handcurl, gripper
RIGHT_ARM_MOTOR_IDS = [15,6,10,11,25,32
    #ID_for_shoulderspin_right,
    #ID_for_biceplift_right,
    #ID_for_elbow_right,
    #ID_for_wristspin_right,
    #ID_for_handcurl_right,
    #ID_for_gripper_right
]

# --- Initialize IK solver state for each arm ---
# These variables will hold the latest joint angles to ensure smooth motion
current_ik_left = np.zeros(len(left_arm_chain.links))
current_ik_right = np.zeros(len(right_arm_chain.links))

# --- Center all motors to their zero positions ---
print("Centering motors...")
for motor_id in LEFT_ARM_MOTOR_IDS + RIGHT_ARM_MOTOR_IDS:
    motor_object = robot.getMotor(motor_id) # Find the correct motor object
    if motor_object:
        motor_object.target = (0, 'P')
    else:
        print(f"Warning: Motor ID {motor_id} not found during centering.")

simStartTime = time.time()
while time.time() - simStartTime < 2:
    robot.moveAllToTarget()
    time.sleep(0.01)
print("Motors centered.")


# ============================================================================
# TRAJECTORY AND IK SETUP
# ============================================================================

# --- Define movement parameters ---
MOVE_DURATION = 15.0  # The movement will take 15 seconds
TARGET_ORIENTATION = np.array([0.0, -1.0, 0.0]) # Point the end-effector's Y-axis down

# --- STEP 1: DEFINE THE PERMANENT OFFSET ---
# The position of the robot's chest (base) in the CoppeliaSim world
ROBOT_BASE_OFFSET = np.array([0.0, 0.0, 0.73889])


# --- STEP 2: APPLY THE OFFSET TO YOUR WORLD TARGETS ---

# --- Define Start and End positions for the LEFT ARM ---
# The start position is calculated with forward kinematics, which is already relative to the robot, so it does not need to be changed.
start_pos_left_fk = left_arm_chain.forward_kinematics(current_ik_left)
start_pos_left = start_pos_left_fk[:3, 3] # Extract [X, Y, Z]

# --- Define Start and End positions for the RIGHT ARM ---
# The start position is already relative to the robot.
start_pos_right_fk = right_arm_chain.forward_kinematics(current_ik_right)
start_pos_right = start_pos_right_fk[:3, 3]

# Define the desired END position in WORLD coordinates
world_end_pos_left1 = np.array([0.15, -0.35, 0.5])
world_end_pos_right1 = np.array([-0.15, -0.35, 0.5])

world_end_pos_left2 = np.array([0.3, -0.3, 0.5])
world_end_pos_right2 = np.array([-0.3, -0.3, 0.5])

world_end_pos_left3 = np.array([0.05, -0.3, 0.5])
world_end_pos_right3 = np.array([-0.05, -0.3, 0.5])
# Subtract the offset to get the end position RELATIVE TO THE ROBOT'S BASE
end_pos_left1 = world_end_pos_left1 - ROBOT_BASE_OFFSET
end_pos_right1 = world_end_pos_right1 - ROBOT_BASE_OFFSET

end_pos_left2 = world_end_pos_left2 - ROBOT_BASE_OFFSET
end_pos_right2 = world_end_pos_right2 - ROBOT_BASE_OFFSET

end_pos_left3 = world_end_pos_left3 - ROBOT_BASE_OFFSET
end_pos_right3 = world_end_pos_right3 - ROBOT_BASE_OFFSET
# --- Create Trajectory Planners for each arm ---
# No changes are needed here, as the planner will now get the correct relative start and end points.
leftArmTrajData1 = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [start_pos_left, end_pos_left1],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
left_traj_planner1 = TrajPlannerTime(*leftArmTrajData1)

rightArmTrajData1 = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [start_pos_right, end_pos_right1],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
right_traj_planner1 = TrajPlannerTime(*rightArmTrajData1)

leftArmTrajData2 = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [end_pos_left1, end_pos_left2],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
left_traj_planner2 = TrajPlannerTime(*leftArmTrajData2)

rightArmTrajData2 = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [end_pos_right1, end_pos_right2],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
right_traj_planner2 = TrajPlannerTime(*rightArmTrajData2)


leftArmTrajData3 = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [end_pos_left2, end_pos_left3],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
left_traj_planner3 = TrajPlannerTime(*leftArmTrajData3)

rightArmTrajData3 = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [end_pos_right2, end_pos_right3],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
right_traj_planner3 = TrajPlannerTime(*rightArmTrajData3)
# ============================================================================
# MAIN EXECUTION LOOP (SIMULTANEOUS ARM MOVEMENT)
# ============================================================================
print(f"Starting sequential arm movement for a total of {MOVE_DURATION * 3} seconds...")
startTime = time.time()

# This loop will now run for the total duration of all three movements
while time.time() - startTime < MOVE_DURATION * 4:
    elapsed_time = time.time() - startTime

    # Use an if/elif/else block to choose the correct trajectory for the current time
    if elapsed_time < MOVE_DURATION:
        # --- Stage 1: First 15 seconds ---
        # The time for this planner is just the elapsed_time
        time_in_segment = elapsed_time
        target_left = left_traj_planner1.getQuinticPositions(time_in_segment)
        target_right = right_traj_planner1.getQuinticPositions(time_in_segment)

    elif elapsed_time < MOVE_DURATION * 2:
        # --- Stage 2: From 15 to 30 seconds ---
        # The time for this planner must be reset to start from 0
        time_in_segment = elapsed_time - MOVE_DURATION
        target_left = left_traj_planner2.getQuinticPositions(time_in_segment)
        target_right = right_traj_planner2.getQuinticPositions(time_in_segment)
        
    else:
        # --- Stage 3: From 30 to 45 seconds ---
        # The time for this planner must also be reset
        time_in_segment = elapsed_time - (MOVE_DURATION * 3)
        target_left = left_traj_planner3.getQuinticPositions(time_in_segment)
        target_right = right_traj_planner3.getQuinticPositions(time_in_segment)

    # --- Now, perform a single IK calculation for the chosen target ---

    # Left Arm Calculation
    ik_solution_left = left_arm_chain.inverse_kinematics(
        target_position=target_left,
        initial_position=current_ik_left,
        target_orientation=TARGET_ORIENTATION,
        orientation_mode="Y"
    )
    current_ik_left = ik_solution_left

    # Right Arm Calculation
    ik_solution_right = right_arm_chain.inverse_kinematics(
        target_position=target_right,
        initial_position=current_ik_right,
        target_orientation=TARGET_ORIENTATION,
        orientation_mode="Y"
    )
    current_ik_right = ik_solution_right


    # --- Send the final commands to the motors ---
    for i, motor_id in enumerate(LEFT_ARM_MOTOR_IDS):
        angle = current_ik_left[i + 1]
        motor_object = robot.getMotor(motor_id)
        if motor_object:
            motor_object.target = (angle, 'P')
        else:
            print(f"Warning: Left arm motor with ID {motor_id} not found.")

    for i, motor_id in enumerate(RIGHT_ARM_MOTOR_IDS):
        angle = current_ik_right[i + 1]
        motor_object = robot.getMotor(motor_id)
        if motor_object:
            motor_object.target = (angle, 'P')
        else:
            print(f"Warning: Right arm motor with ID {motor_id} not found.")

    # Move All Motors
    robot.moveAllToTarget()
    
    time.sleep(0.01)

print("Sequential movement finished!")
