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
world_end_pos_left = np.array([0.15, -0.35, 0.5])
world_end_pos_right = np.array([-0.15, -0.35, 0.5])

# Subtract the offset to get the end position RELATIVE TO THE ROBOT'S BASE
end_pos_left = world_end_pos_left - ROBOT_BASE_OFFSET
end_pos_right = world_end_pos_right - ROBOT_BASE_OFFSET



# --- Create Trajectory Planners for each arm ---
leftArmTrajData = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [start_pos_left, end_pos_left],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
left_traj_planner = TrajPlannerTime(*leftArmTrajData)

rightArmTrajData = [
    [[0] * 3, [MOVE_DURATION] * 3],
    [start_pos_right, end_pos_right],
    [[0] * 3, [0] * 3],
    [[0] * 3, [0] * 3]
]
right_traj_planner = TrajPlannerTime(*rightArmTrajData)

# ============================================================================
# MAIN EXECUTION LOOP (SIMULTANEOUS ARM MOVEMENT)
# ============================================================================
print(f"Starting simultaneous arm movement for {MOVE_DURATION} seconds...")
startTime = time.time()

while time.time() - startTime < MOVE_DURATION:
    elapsed_time = time.time() - startTime

    # --- Left Arm Calculation ---
    target_left = left_traj_planner.getQuinticPositions(elapsed_time)
    ik_solution_left = left_arm_chain.inverse_kinematics(
        target_position=target_left,
        initial_position=current_ik_left,
        target_orientation=TARGET_ORIENTATION,
        orientation_mode="Y"
    )
    current_ik_left = ik_solution_left

    # --- Right Arm Calculation ---
    target_right = right_traj_planner.getQuinticPositions(elapsed_time)
    ik_solution_right = right_arm_chain.inverse_kinematics(
        target_position=target_right,
        initial_position=current_ik_right,
        target_orientation=TARGET_ORIENTATION,
        orientation_mode="Y"
    )
    current_ik_right = ik_solution_right

    # --- Send Commands to Motors ---
    # Command LEFT arm motors
    for i, motor_id in enumerate(LEFT_ARM_MOTOR_IDS):
        angle = current_ik_left[i + 1]  # +1 to skip the non-motorized base link
        motor_object = robot.getMotor(motor_id) # Find the motor in the list
        if motor_object:
            motor_object.target = (angle, 'P')
        else:
            print(f"Warning: Left arm motor with ID {motor_id} not found.")


    # Command RIGHT arm motors
    for i, motor_id in enumerate(RIGHT_ARM_MOTOR_IDS):
        angle = current_ik_right[i + 1]
        motor_object = robot.getMotor(motor_id) # Find the motor in the list
        if motor_object:
            motor_object.target = (angle, 'P')
        else:
            print(f"Warning: Right arm motor with ID {motor_id} not found.")


    # --- Move All Motors ---
    # This single command executes the movements for both arms at once
    robot.moveAllToTarget()
    
    time.sleep(0.01)

print("Dual arm movement finished!")