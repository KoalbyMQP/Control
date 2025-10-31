import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import sys, time, math, array
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
# from backend.Testing import finlyViaPoints as via


right_arm_chain = Chain.from_urdf_file(
    "backend/Testing/FinleyJNEWARMS_2024_straight_4.urdf",
    base_elements=['shoulder1_right', 'shoulder1_right'],
    active_links_mask=[False, True, True, True, True, True, True]  
)

# Creating URDF chain for left arm 
left_arm_chain = Chain.from_urdf_file(
    "backend/Testing/FinleyJNEWARMS_2024_straight_4.urdf",
    base_elements=['shoulder1_left', 'shoulder1_left'],
    active_links_mask=[False, True, True, True, True, True, True]  
)

# creating URDF chain for camera
camera = Chain.from_urdf_file(
    "backend/Testing/FinleyJNEWARMS_2024_straight_4.urdf",
    base_elements=['neck', 'neck']   
)

INITIAL_ARM_ANGLES = np.array([0.0] * 7)
left_arm_angles = INITIAL_ARM_ANGLES.copy()
right_arm_angles = INITIAL_ARM_ANGLES.copy()

#forward kinematics for camera chain
camera_angles=np.array([0,0,0,0])
camera_frame_transformation=camera.forward_kinematics(camera_angles)


# Edit to declare if you are testing the sim or the real robot
is_real = False
robot = Robot(is_real)
print("Setup Complete")


# positions


final_position=np.array([0,  0, 0])

start_to_finish_time = time.time()
init_start = time.time()


#Starting Agnles

# robot.motors[25].target = (math.radians(30), 'P')
# robot.motors[26].target = (math.radians(10), 'P')


robot.motors[5].target = (math.radians(0), 'P') #shoulder 1 koint - -180 to 180
robot.motors[6].target = (math.radians(0), 'P') #shoulder 2 joint - 85 sends it down to legs, -90 sends arm above head
robot.motors[7].target = (math.radians(0), 'P') #Elbow joint - 110 moves towards board, may have overlap with link, -110 works
robot.motors[8].target = (math.radians(0), 'P') #Forearm joint - 180 to -180 should work
robot.motors[9].target = (math.radians(0), 'P') #Wrist joint - 90 works, but we don't need it to bend in that angle, -130 is maximum
robot.motors[10].target = (math.radians(0), 'P') #nothing? will just do -180 to 180

robot.motors[0].target = (math.radians(0), 'P') #shoulder 1 koint - -180 to 180
robot.motors[1].target = (math.radians(0), 'P') #shoulder 2 joint - 85 sends it down to legs, -90 sends arm above head
robot.motors[2].target = (math.radians(0), 'P') #Elbow joint - 110 moves towards board, may have overlap with link, -110 works
robot.motors[3].target = (math.radians(0), 'P') #Forearm joint - 180 to -180 should work
robot.motors[4].target = (math.radians(0), 'P') #Wrist joint - 90 works, but we don't need it to bend in that angle, -130 is maximum
robot.motors[11].target = (math.radians(0), 'P') #nothing? will just do -180 to 180


ik_solution_2=np.array([0,0,0,0,0,0,0])


# centering all angles to zero
prevTime = time.time()
simStartTime = time.time()
while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()

init_time = time.time() - init_start
print(f"Initialize time: {init_time:.6f} seconds")

entities_start = time.time()


# conversion of final points from camera coordinate systm to rorbot coordinate system 
final_points=np.array([0, 0, 0])
B=np.array([[final_points[0]],[final_points[1]],[final_points[2]],[1]])
A= camera_frame_transformation
final_points=np.array([0,0, 0])
C = np.dot(A, B)


# print(C)


end_effector_start_pos = [.49076,  -.08197, .76541]
# end_effector_start_pos = [0.1151, -0.08197, 0.39374]
end_effector_z_offset = 0.1
end_effector_y_offset = 0.0
end_effector_x_offset = 0.007
end_position = [0.128+end_effector_x_offset, -0.200+end_effector_y_offset, 0.615+end_effector_z_offset]


leftArmTraj = [
    [[0,0,0], [20, 20, 20]],
    [[end_effector_start_pos[0],  end_effector_start_pos[1], end_effector_start_pos[2]],
   [end_position[0]+end_effector_x_offset, end_position[1]+end_effector_y_offset, end_position[2]+end_effector_z_offset]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]

final_position=left_arm_chain.forward_kinematics(ik_solution_2)

lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])

sim_run_start = time.time()

def within_threshold(p1, p2, threshold):
    p1 = np.asarray(p1).reshape(-1)
    p2 = np.asarray(p2).reshape(-1)
    dist = np.linalg.norm(p1 - p2)
    return dist <= threshold, dist

threshold = 3
alpha = 0.5

startTime = time.time()
target_orientation = np.array([0.0, -1.0, 1.0])

JOINT_LIMITS = {
    5: (np.deg2rad(-180), np.deg2rad(180)),
    6: (np.deg2rad(-90), np.deg2rad(130)), #100
    7: (np.deg2rad(-110), np.deg2rad(110)),  
    8: (np.deg2rad(-180), np.deg2rad(180)),
    9: (np.deg2rad(-130), np.deg2rad(90))
}

def execute_trajectory(chain, robot, ik_init, target_orientation,
                       start_pos, end_pos, duration, motor_ids, threshold=None, safety_margin=0.01):
    # Define the trajectory
    leftArmTraj = [
        [[0, 0, 0], [duration, duration, duration]],  # timing vector
        [start_pos, end_pos],                         # positions
        [[0, 0, 0], [0, 0, 0]],                       # initial velocities
        [[0, 0, 0], [0, 0, 0]]                        # final velocities
    ]
    traj_planner = TrajPlannerTime(leftArmTraj[0],
                                   leftArmTraj[1],
                                   leftArmTraj[2],
                                   leftArmTraj[3])

    # Initialize loop
    start_time = time.time()
    ik_solution_2 = ik_init
    trajectory_buffer = []
    ik_recalc_count = 0
    ik_time_total = 0.0
    waypoint_count = 0

    while time.time() - start_time < duration:
        target_position_task = traj_planner.getQuinticPositions(time.time() - start_time)
        target_position = np.array(target_position_task[:3])  # just x,y,z
        # Solve IK
        ik_start = time.time()
        ik_solution = chain.inverse_kinematics(
            target_position,
            initial_position=ik_solution_2,
            target_orientation=target_orientation,
            orientation_mode="Y",
            regularization_parameter=0.01
        )
        ik_time_total += time.time() - ik_start
        waypoint_count += 1

        if threshold is not None:
            fk_solution = chain.forward_kinematics(ik_solution)
            fk_coords = [fk_solution[0][3], fk_solution[1][3], fk_solution[2][3]]
            ok, _ = within_threshold(target_position, fk_coords, threshold)
            if not ok:
                # print("Reinitializing IK")
                ik_recalc_count += 1
                ik_solution = chain.inverse_kinematics(
                    fk_coords,
                    initial_position=ik_solution_2,
                    target_orientation=target_orientation,
                    orientation_mode="Y",
                    regularization_parameter=0.01
                )

        # for j, angle in enumerate(ik_solution):
        #     if j in JOINT_LIMITS:
        #         min_angle, max_angle = JOINT_LIMITS[j]
        #         if not (min_angle + safety_margin <= angle <= max_angle - safety_margin):
        #             print(f"Joint {j} out of range: {angle:.3f} rad "
        #                   f"(limit {min_angle:.3f} {max_angle:.3f}). Stopping.")
        #             return ik_solution_2, trajectory_buffer

        # Update guess and buffer
        ik_solution_2 = ik_solution
        trajectory_buffer.append(ik_solution)

        # Send to motors
        for idx, motor_id in enumerate(motor_ids, start=1):
            if idx < len(ik_solution):
                robot.motors[motor_id].target = (ik_solution[idx], 'P')
            else:
                print(f"Warning: Motor ID {motor_id} has no corresponding IK solution index.")

        robot.moveAllToTarget()
        # total_time = time.time() - start_time
        # print(f"\n--- Trajectory Stats ---")
        # print(f"Waypoints generated: {waypoint_count}")
        # print(f"IK recalculations:   {ik_recalc_count}")
        # print(f"Total runtime:       {total_time:.4f} s")
        # print(f"Total IK time:       {ik_time_total:.4f} s")
        # print(f"Avg IK per waypoint: {ik_time_total / waypoint_count:.6f} s\n")
    return ik_solution_2, trajectory_buffer

MOTOR_ID_MAP = {
    "left": [5, 6, 7, 8, 9, 10], 
    "right": [0, 1, 2, 3, 4, 10]
}

target_position_world = np.array([-0.095, -0.460, 0.76541])

left_fk_matrices = left_arm_chain.forward_kinematics(INITIAL_ARM_ANGLES, full_kinematics=True)
print(left_fk_matrices)
left_shoulder_pos = left_fk_matrices[0][:3, 3] # Position of the base link

right_fk_matrices = right_arm_chain.forward_kinematics(INITIAL_ARM_ANGLES, full_kinematics=True)
right_shoulder_pos = right_fk_matrices[0][:3, 3]

print(f"Target: {target_position_world}")
print(f"Left shoulder: {left_shoulder_pos}")
print(f"Right shoulder: {right_shoulder_pos}")


dist_left = np.linalg.norm(target_position_world - left_shoulder_pos)
dist_right = np.linalg.norm(target_position_world - right_shoulder_pos)

print(f"Distance to left: {dist_left:.3f}, Distance to right: {dist_right:.3f}")

if dist_left <= dist_right:
    print("Target is closer to LEFT arm. Moving left arm.")
    chosen_chain = left_arm_chain
    chosen_motor_ids = MOTOR_ID_MAP["left"]
    current_arm_angles = left_arm_angles
else:
    print("Target is closer to RIGHT arm. Moving right arm.")
    chosen_chain = right_arm_chain
    chosen_motor_ids = MOTOR_ID_MAP["right"]
    current_arm_angles = right_arm_angles

new_arm_angles, traj1 = execute_trajectory(
    left_arm_chain,
    robot,
    ik_solution_2,
    target_orientation,
    start_pos=end_effector_start_pos,
    end_pos=target_position_world,
    duration=5,
    motor_ids=chosen_motor_ids,
    threshold=4
)

# # 8. Update the state for the arm that just moved
# if chosen_chain == left_arm_chain:
#     left_arm_angles = new_arm_angles
#     print("Updated left arm angles.")
# else:
#     right_arm_angles = new_arm_angles
#     print("Updated right arm angles.")

# sim_run_time = time.time() - sim_run_start
# print(f"Sim Run Time: {sim_run_time:.6f} seconds")

# total_time = time.time() - start_to_finish_time
# print(f"\nStart to finish: {total_time:.6f} seconds")

# pick_up_position = [end_position[0], end_position[1], end_position[2]-(end_effector_z_offset/2)]
# end_board_position = [end_position[0], end_position[1]+0.1, end_position[2]]

# ik_solution_2, traj2 = execute_trajectory(
#     left_arm_chain,
#     robot,
#     ik_solution_2,
#     target_orientation,
#     start_pos=end_position,
#     end_pos=pick_up_position,
#     duration=5,
#     threshold=4
# )

# ik_solution_2, traj3 = execute_trajectory(
#     left_arm_chain,
#     robot,
#     ik_solution_2,
#     target_orientation,
#     start_pos=pick_up_position,
#     end_pos=end_position,
#     duration=5,
#     threshold=4
# )

# ik_solution_2, traj4 = execute_trajectory(
#     left_arm_chain,
#     robot,
#     ik_solution_2,
#     target_orientation,
#     start_pos=end_position,
#     end_pos=end_position,
#     duration=5,
#     threshold=4
# )


# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------

# import numpy as np
# import time

# # --- Configuration Parameters ---
# # Define these at the top of your script
# INTERMEDIATE_POS = np.array([0.0, 0.3, 0.4])  # A safe "home" position
# OFF_BOARD_POS = np.array([0.4, 0.0, 0.2])     # Position to drop captured pieces
# Z_OFFSET = 0.08                              # Vertical distance for safe approach (in meters)

# # --- Gripper Control (Placeholder) ---
# # Replace these with your actual gripper commands
# def grip(robot):
#     print("ACTION: Gripper Closing")
#     # your_robot.close_gripper()
#     time.sleep(1)

# def release(robot):
#     print("ACTION: Gripper Opening")
#     # your_robot.open_gripper()
#     time.sleep(1)

# def perform_pick_or_place(action, target_pos, current_ik, robot, chain, target_orientation):
#     #Move to a safe position directly above the target
#     safe_approach_pos = target_pos + np.array([0, 0, Z_OFFSET])
    
#     # Get current position from FK
#     fk_solution = chain.forward_kinematics(current_ik)
#     start_pos = np.array([fk_solution[0][3], fk_solution[1][3], fk_solution[2][3]])
    
#     ik_after_approach, _ = execute_trajectory(
#         chain, robot, current_ik, target_orientation,
#         start_pos=start_pos, end_pos=safe_approach_pos, duration=3
#     )

#     #Move straight down to the target
#     ik_after_descend, _ = execute_trajectory(
#         chain, robot, ik_after_approach, target_orientation,
#         start_pos=safe_approach_pos, end_pos=target_pos, duration=2
#     )

#     #Perform the gripper action
#     if action == 'pick':
#         grip(robot)
#     elif action == 'place':
#         release(robot)

#     #Move straight back up to the safe position
#     ik_after_ascend, _ = execute_trajectory(
#         chain, robot, ik_after_descend, target_orientation,
#         start_pos=target_pos, end_pos=safe_approach_pos, duration=2
#     )
    
#     return ik_after_ascend


# def handle_move_piece(piece_loc, goal_loc, current_ik, robot, chain, target_orientation):
#     #Pick up the piece from its starting location
#     ik_after_pick = perform_pick_or_place(
#         'pick', piece_loc, current_ik, robot, chain, target_orientation
#     )
    
#     #Place the piece at the goal location
#     ik_after_place = perform_pick_or_place(
#         'place', goal_loc, ik_after_pick, robot, chain, target_orientation
#     )

#     #Return to the intermediate 'Idle' position
#     fk_solution = chain.forward_kinematics(ik_after_place)
#     start_pos = np.array([fk_solution[0][3], fk_solution[1][3], fk_solution[2][3]])
    
#     final_ik, _ = execute_trajectory(
#         chain, robot, ik_after_place, target_orientation,
#         start_pos=start_pos, end_pos=INTERMEDIATE_POS, duration=4
#     )
    
#     return final_ik

# def handle_capture_piece(your_piece_loc, capture_loc, current_ik, robot, chain, target_orientation):
#     """
#     Handles the 'Capture Piece' state logic.
#     1. Removes the opponent's piece from the board.
#     2. Moves your piece to the now-empty capture location.
#     Returns the final IK solution after returning to the intermediate position.
#     """
#     print(f"\n--- EXECUTING: CAPTURE PIECE ---")
    
#     # Part 1: Remove the opponent's piece
#     print("Step 1: Removing opponent's piece from capture location...")
#     ik_after_removal = perform_pick_or_place(
#         'pick', capture_loc, current_ik, robot, chain, target_orientation
#     )
#     print("\nStep 2: Placing opponent's piece off-board...")
#     ik_after_drop = perform_pick_or_place(
#         'place', OFF_BOARD_POS, ik_after_removal, robot, chain, target_orientation
#     )

#     # Part 2: Move your piece to the capture location
#     print("\nStep 3: Picking up your piece...")
#     ik_after_pick = perform_pick_or_place(
#         'pick', your_piece_loc, ik_after_drop, robot, chain, target_orientation
#     )
#     print("\nStep 4: Placing your piece at capture location...")
#     ik_after_place = perform_pick_or_place(
#         'place', capture_loc, ik_after_pick, robot, chain, target_orientation
#     )

#     # Part 3: Return to the intermediate 'Idle' position
#     print("\nStep 5: Returning to Idle position...")
#     fk_solution = chain.forward_kinematics(ik_after_place)
#     start_pos = np.array([fk_solution[0][3], fk_solution[1][3], fk_solution[2][3]])

#     final_ik, _ = execute_trajectory(
#         chain, robot, ik_after_place, target_orientation,
#         start_pos=start_pos, end_pos=INTERMEDIATE_POS, duration=4
#     )
    
#     print("--- CAPTURE PIECE COMPLETE ---")
#     return final_ik
