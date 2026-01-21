import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import sys, time, math, array
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.KoalbyHumanoid.ConfigKoalby2 import Joints
from backend.KoalbyHumanoid.ConfigKoalby2 import Joints


class KoalbyArmController:
    """
    A class to control the Koalby Humanoid's arms using IK and trajectories.
    """
    
    def __init__(self, is_real=False, urdf_path="backend/Testing/Humanoid_URDF_9-10.urdf"):
        self.is_real = is_real
        self.urdf_path = urdf_path
        
        # Initialize Robot
        print("Initializing Robot...")
        self.robot = Robot2(is_real)
        print("Robot instance created.")

        # Create URDF chains
        print("Loading IK chains from URDF...")
        self.right_arm_chain = Chain.from_urdf_file(
            self.urdf_path,
            base_elements=['shoulder_right', 'shoulderspin_right'],
            active_links_mask=[False, True, True, True, True, True, True]
        )
        
        self.left_arm_chain = Chain.from_urdf_file(
            self.urdf_path,
            base_elements=['shoulder_left', 'shoulderspin_left'],
            active_links_mask=[False, True, True, True, True, True, True]
        )

        self.camera_chain = Chain.from_urdf_file(
            self.urdf_path,
            base_elements=['neck', 'neckturn']
        )

        print("IK chains loaded.")

        # Initial state
        # self.INITIAL_ARM_ANGLES = np.array([0.0] * 7)
        self.INITIAL_ARM_ANGLES = np.array([
            0.0,                # 0: Base link (inactive)
            0.0,                # 1: Shoulder spin (motor 6)
            np.deg2rad(0),      # 2: Shoulder lift (motor 7)
            0.0,                # 3: Elbow joint (motor 8) <-- KEY CHANGE
            0.0,                # 4: Forearm spin (motor 9)
            0.0,                # 5: Wrist (motor 10)
            0.0                 # 6: Gripper (motor 11)
        ])

        self.left_arm_angles = self.INITIAL_ARM_ANGLES.copy()
        self.right_arm_angles = self.INITIAL_ARM_ANGLES.copy()

        self.camera_angles = [0, 0, 0, 0]
        
        # Forward kinematics for camera
        self.camera_frame_transformation = self.camera_chain.forward_kinematics(self.camera_angles, False)

        # Constants
        # Joint limits based on your original script
        self.JOINT_LIMITS = {
            # Left Arm
            Joints.shoulderspin_left: (np.deg2rad(-180), np.deg2rad(180)),
            Joints.biceplift_left: (np.deg2rad(-90), np.deg2rad(130)), #100
            Joints.elbow_left: (np.deg2rad(-110), np.deg2rad(110)),  
            Joints.wristspin_left: (np.deg2rad(-180), np.deg2rad(180)),
            Joints.handcurl_left: (np.deg2rad(-130), np.deg2rad(90)),
            Joints.gripper_left: (np.deg2rad(-180), np.deg2rad(180)), # From init loop

            # Right Arm
            Joints.shoulderspin_right: (np.deg2rad(-180), np.deg2rad(180)),
            Joints.biceplift_right: (np.deg2rad(-90), np.deg2rad(130)), #100
            Joints.elbow_right: (np.deg2rad(-110), np.deg2rad(110)),  
            Joints.wristspin_right: (np.deg2rad(-180), np.deg2rad(180)),
            Joints.handcurl_right: (np.deg2rad(-130), np.deg2rad(90)), 
            Joints.gripper_right: (np.deg2rad(-180), np.deg2rad(180)) # From init loop
        }
        
        # Motor mapping based on your init loop
        self.MOTOR_ID_MAP = {
            "left": [Joints.shoulderspin_left,
                     Joints.biceplift_left, 
                     Joints.elbow_left, 
                     Joints.wristspin_left, 
                     Joints.handcurl_left,
                     Joints.gripper_left],

            "right": [Joints.shoulderspin_right,
                      Joints.biceplift_right, 
                      Joints.elbow_right, 
                      Joints.wristspin_right, 
                      Joints.handcurl_right,
                      Joints.gripper_right]
        }

        self.GRIPPER_OPEN_VAL = math.radians(-60)        
        self.GRIPPER_CLOSED_VAL = math.radians(5) #overclose the gripper to get a grasp
        
        # Track state: 'left' and 'right'
        self.gripper_states = {
            "left": self.GRIPPER_OPEN_VAL,
            "right": self.GRIPPER_OPEN_VAL
        }
        
        # Store the last IK solution as the initial guess for the next
        self.ik_solution_left = self.INITIAL_ARM_ANGLES.copy()
        self.ik_solution_right = self.INITIAL_ARM_ANGLES.copy()

        print("KoalbyArmController setup complete.")

    def set_gripper(self, arm_side, value):
        """
        Manually sets gripper target and updates internal state.
        """
        self.gripper_states[arm_side] = value
        
        if arm_side == "left":
            motor_idx = Joints.gripper_left
            motor_idx = Joints.gripper_left
        else: 
            motor_idx = Joints.gripper_right
            
        print(f"Setting {arm_side} gripper (Motor {motor_idx}) to {value:.3f}")
        self.robot.motors[motor_idx].target = (value, 'P')
        self.robot.moveAllToTarget()
        time.sleep(0.5) 

    def set_shoulder(self, arm_side, value):
        """
        Sets the shoulder angle manually and UPDATES the IK seed 
        so the next trajectory doesn't snap it back.
        """
        # 1. Identify indices based on your URDF/Array structure
        # Index 1 in ik_solution corresponds to the Shoulder Spin (Motor 6/0)
        # Index 2 in ik_solution corresponds to the Shoulder Lift (Motor 7/1)
        # Assuming you want to change the SHOULDER SPIN (Motor 6 or 0):
        ik_index = 0
        
        if arm_side == "left":
            motor_id = Joints.shoulderspin_left
            motor_id = Joints.shoulderspin_left
            self.ik_solution_left[ik_index] = value
        elif arm_side == "right":
            motor_id = Joints.shoulderspin_right
            motor_id = Joints.shoulderspin_right
            self.ik_solution_right[ik_index] = value
        else:
            return

        print(f"Manually setting {arm_side} shoulder (Motor {motor_id}) to {math.degrees(value):.1f} deg")
        
        # 2. Move the physical motor
        self.robot.motors[motor_id].target = (value, 'P')
        self.robot.moveAllToTarget()
        time.sleep(0.5) # Allow time to reach position

    def open_gripper(self, arm_side):
        self.set_gripper(arm_side, self.GRIPPER_OPEN_VAL)

    def close_gripper(self, arm_side):
        self.set_gripper(arm_side, self.GRIPPER_CLOSED_VAL)

    def initialize_robot_position(self):
        self.robot.motors[Joints.shoulderspin_left].target = (math.radians(0), 'P') 
        self.robot.motors[Joints.biceplift_left].target = (math.radians(0), 'P') 
        self.robot.motors[Joints.elbow_left].target = (math.radians(0), 'P') 
        self.robot.motors[Joints.wristspin_left].target = (math.radians(0), 'P') 
        self.robot.motors[Joints.handcurl_left].target = (math.radians(0), 'P') 
        self.robot.motors[Joints.gripper_left].target = (math.radians(0), 'P') 
        self.robot.motors[Joints.shoulderspin_right].target = (math.radians(0), 'P') #shoulder 1 koint - -180 to 180
        self.robot.motors[Joints.biceplift_right].target = (math.radians(0), 'P') #shoulder 2 joint - 85 sends it down to legs, -90 sends arm above head
        self.robot.motors[Joints.elbow_right].target = (math.radians(0), 'P') #Elbow joint - 110 moves towards board, may have overlap with link, -110 works
        self.robot.motors[Joints.wristspin_right].target = (math.radians(0), 'P') #Forearm joint - 180 to -180 should work
        self.robot.motors[Joints.handcurl_right].target = (math.radians(0), 'P') #Wrist joint - 90 works, but we don't need it to bend in that angle, -130 is maximum
        self.robot.motors[Joints.gripper_right].target = (math.radians(0), 'P') #nothing? will just do -180 to 180
        self.robot.moveAllToTarget()

    def transform_camera_to_robot(self, camera_point):
        """
        Converts a 3D point from the camera coordinate system to the robot's base coordinate system.
        :param camera_point: A list or numpy array of [x, y, z].
        :return: A numpy array [x, y, z] in the robot's coordinate system.
        """
        # Create a 4x1 homogeneous coordinates vector
        B = np.array([[camera_point[0]], [camera_point[1]], [camera_point[2]], [1]])
        # Apply the transformation matrix
        C = np.dot(self.camera_frame_transformation, B)
        # Return just the [x, y, z] part
        return C[:3].flatten()

    @staticmethod
    def within_threshold(p1, p2, threshold):
        """
        Checks if the Euclidean distance between two points is within a threshold.
        :param p1: First point [x, y, z].
        :param p2: Second point [x, y, z].
        :param threshold: Distance threshold.
        :return: (bool, float) - (is_within_threshold, distance)
        """
        p1 = np.asarray(p1).reshape(-1)
        p2 = np.asarray(p2).reshape(-1)
        dist = np.linalg.norm(p1 - p2)
        return dist <= threshold, dist

    def execute_arm_trajectory(self, arm_side, start_pos, end_pos, duration, 
                               target_orientation, ik_threshold=None, check_joint_limits = False, safety_margin=0.01, orientation_mode="Y"):
        
        if arm_side == "left":
            chain = self.left_arm_chain
            motor_ids = self.MOTOR_ID_MAP["left"]
            ik_solution_prev = self.ik_solution_left
            print("IK Solution Prev:", ik_solution_prev)

        elif arm_side == "right":
            chain = self.right_arm_chain
            motor_ids = self.MOTOR_ID_MAP["right"]
            ik_solution_prev = self.ik_solution_right

        else:
            raise ValueError("arm_side must be 'left' or 'right'")

        # Define the trajectory parameters for TrajPlannerTime
        arm_traj_params = [
            [[0, 0, 0], [duration, duration, duration]],  # timing vector
            [start_pos, end_pos],                         # positions
            [[0, 0, 0], [0, 0, 0]],                       # initial velocities
            [[0, 0, 0], [0, 0, 0]]                        # final velocities
        ]
        
        try:
            traj_planner = TrajPlannerTime(arm_traj_params[0],
                                           arm_traj_params[1],
                                           arm_traj_params[2],
                                           arm_traj_params[3])
            
        except Exception as e:
            print(f"Error creating TrajPlannerTime: {e}")
            return ik_solution_prev, []

        # Initialize loop
        start_time = time.time()
        trajectory_buffer = []
        ik_recalc_count = 0
        ik_time_total = 0.0
        waypoint_count = 0
        
        print(f"Executing {arm_side} arm trajectory for {duration} seconds...")
        print(f"  From: {np.round(start_pos, 3)}")
        print(f"  To:   {np.round(end_pos, 3)}")

        desired_gripper_val = self.gripper_states[arm_side]

        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            target_position_task = traj_planner.getQuinticPositions(elapsed_time)
            target_position = np.array(target_position_task[:3])  # just x,y,z
            
            # Solve IK
            ik_start = time.time()
            try:
                ik_solution = chain.inverse_kinematics(
                    target_position,
                    initial_position=ik_solution_prev,
                    target_orientation=target_orientation,
                    orientation_mode=orientation_mode,
                    regularization_parameter=0.01
                )
            except ValueError as e:
                # This can happen if the target is unreachable
                print(f"IK calculation error at t={elapsed_time:.2f}s: {e}")
                print(f"  Target: {target_position}, Initial Guess: {ik_solution_prev}")
                print("  Stopping trajectory due to IK error.")
                break # Exit the while loop
            
            ik_time_total += time.time() - ik_start
            waypoint_count += 1

            # Check threshold if provided
            if ik_threshold is not None:
                fk_solution = chain.forward_kinematics(ik_solution)
                fk_coords = [fk_solution[0][3], fk_solution[1][3], fk_solution[2][3]]
                ok, _ = self.within_threshold(target_position, fk_coords, ik_threshold)
                if not ok:
                    # print("Reinitializing IK")
                    ik_recalc_count += 1
                    ik_solution = chain.inverse_kinematics(
                        fk_coords,
                        initial_position=ik_solution_prev,
                        target_orientation=target_orientation,
                        orientation_mode="Y",
                        regularization_parameter=0.01
                    )

            # Check joint limits
            if check_joint_limits == True:
                joint_limit_exceeded = False
                # ik_solution[0] is the base link, arm joints start at index 1
                for j, angle in enumerate(ik_solution[1:], start=0):
                    if j >= len(motor_ids):
                        break # Should not happen if active_links_mask is correct
                    
                    motor_id = motor_ids[j] 
                    if motor_id in self.JOINT_LIMITS:
                        min_angle, max_angle = self.JOINT_LIMITS[motor_id]
                        if not (min_angle + safety_margin <= angle <= max_angle - safety_margin):
                            print(f"Joint {motor_id} (idx {j+1}) out of range: {math.degrees(angle):.1f} deg "
                                f"(limit {math.degrees(min_angle):.1f} {math.degrees(max_angle):.1f}). Stopping.")
                            joint_limit_exceeded = True
                            break
                if joint_limit_exceeded:
                    break

            # Update guess and buffer
            ik_solution_prev = ik_solution
            # trajectory_buffer.append(ik_solution)

            # Send to motors
            # ik_solution[0] is base link, so ik_solution[1] maps to motor_ids[0]
            for idx, motor_index in enumerate(motor_ids, start=1):
                if idx < len(ik_solution):
                    if 0 <= motor_index < len(self.robot.motors):
                        if motor_index == Joints.gripper_left or motor_index == Joints.gripper_right:
                            self.robot.motors[motor_index].target = (desired_gripper_val, 'P')
                        else:
                            self.robot.motors[motor_index].target = (ik_solution[idx], 'P')
                    else:
                        print(f"Warning: Motor index {motor_index} out of range for robot.motors list.")
                else:
                    # This should not happen if active_links_mask matches motor_indices
                    print(f"Warning: IK solution index {idx} is out of range.")

            self.robot.moveAllToTarget()
            
            # Optional: Add a small sleep to not overwhelm the simulation/robot
            # time.sleep(0.001) 

        total_time = time.time() - start_time
        print(f"\n--- {arm_side.capitalize()} Trajectory Stats ---")
        print(f"Waypoints generated: {waypoint_count}")
        print(f"IK recalculations:   {ik_recalc_count}")
        print(f"Total runtime:       {total_time:.4f} s")
        if waypoint_count > 0:
            print(f"Total IK time:       {ik_time_total:.4f} s")
            print(f"Avg IK per waypoint: {ik_time_total / waypoint_count:.6f} s\n")

        # Save the last successful IK solution as the new default guess
        if arm_side == "left":
            self.ik_solution_left = ik_solution_prev
        else:
            self.ik_solution_right = ik_solution_prev
            
        return ik_solution_prev, trajectory_buffer

# -----------------------------------------------------------------
# Example Usage
# -----------------------------------------------------------------
if __name__ == "__main__":
        
    is_real_robot = False
    chest_x = 0
    chest_y = 0.05177
    chest_z = 0.73889
    chest_offsets = [chest_x, chest_y, chest_z]
    controller = KoalbyArmController(is_real=is_real_robot, 
                                     urdf_path="backend/Testing/Humanoid_URDF_9-10.urdf")
    
    # 1. Initialize Robot to zero position
    target_orientation = np.array([0, -1, 1])
    init_start = time.time()
    controller.initialize_robot_position()
    # chosen_motor_ids = MOTOR_ID_MAP["left"]
    end_effector_start_pos = [0.49634, -0.06063, 0.03032]
    # end_effector_z_offset = 0.1
    # end_effector_y_offset = 0.0
    # end_effector_x_offset = 0.007 
    # end_position = [0.128, -0.368, -0.127]
    end_position = [0.128, -0.175, -0.127]
    intermediate_end_pos = [end_position[0], end_position[1], end_position[2] + 0.05]
    trajectory_duration = 3


    right_end_effector_start_pos = [-0.49634, -0.04137, 0.03032]
    right_first_end = [-0.131, -0.175, -0.127]
    right_end_pos = [-0.131, -0.368, -0.127]
    right_intermediate_pos = [right_first_end[0], right_first_end[1], right_first_end[2] + 0.05]
    right_intermediate_end_pos = [right_end_pos[0], right_end_pos[1], right_end_pos[2] + 0.05]

    final_ik_right, right_trajectory = controller.execute_arm_trajectory(
        arm_side="right",
        start_pos=right_end_effector_start_pos,
        end_pos=right_intermediate_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )

    final_ik_right, right_trajectory = controller.execute_arm_trajectory(
        arm_side="right",
        start_pos=right_intermediate_pos,
        end_pos=right_first_end,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )

    final_ik_right, right_trajectory = controller.execute_arm_trajectory(
        arm_side="right",
        start_pos=right_first_end,
        end_pos=right_intermediate_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )

    final_ik_right, right_trajectory = controller.execute_arm_trajectory(
        arm_side="right",
        start_pos=right_intermediate_pos,
        end_pos=right_intermediate_end_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )

    final_ik_right, right_trajectory = controller.execute_arm_trajectory(
        arm_side="right",
        start_pos=right_intermediate_end_pos,
        end_pos=right_end_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )

    # -------------------------------------------------------------------------------------------------------------------
    # -------------------------------------------------------------------------------------------------------------------
    # -------------------------------------------------------------------------------------------------------------------
    # -------------------------------------------------------------------------------------------------------------------

    final_ik_left, left_trajectory = controller.execute_arm_trajectory(
        arm_side="left",
        start_pos=end_effector_start_pos,
        end_pos=intermediate_end_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )
    final_ik_left, left_trajectory = controller.execute_arm_trajectory(
        arm_side="left",
        start_pos=intermediate_end_pos,
        end_pos=end_position,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )
    new_end_position = [0.128, -0.368, -0.127]
    final_ik_left, left_trajectory = controller.execute_arm_trajectory(
        arm_side="left",
        start_pos=end_position,
        end_pos=intermediate_end_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0 
    )
    piece_intermediate_end_pos = [new_end_position[0], new_end_position[1], new_end_position[2] + 0.05]
    final_ik_left, left_trajectory = controller.execute_arm_trajectory(
        arm_side="left",
        start_pos=intermediate_end_pos,
        end_pos=piece_intermediate_end_pos,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0
    )
    final_ik_left, left_trajectory = controller.execute_arm_trajectory(
        arm_side="left",
        start_pos=piece_intermediate_end_pos,
        end_pos=new_end_position,
        duration=trajectory_duration,
        target_orientation=target_orientation,
        ik_threshold=3.0 
    )

# ------------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------------

# if __name__ == "__main__":
        
#     is_real_robot = False
#     chest_x = 0
#     chest_y = 0.05177
#     chest_z = 0.73889
#     chest_offsets = [chest_x, chest_y, chest_z]
    
#     print("--- Initializing Controller ---")
#     controller = KoalbyArmController(is_real=is_real_robot, 
#                                      urdf_path="backend/Testing/Humanoid_URDF_9-10.urdf")
    
#     # Define trajectory parameters
#     # end_effector_start_pos = [0.49634 - chest_offsets[0], -0.00885 - chest_offsets[1], 0.76921 - chest_offsets[2]]
#     # end_position = [0.128 - chest_offsets[0], -0.368 - chest_offsets[1], 0.615 - chest_offsets[2]]
#     end_effector_start_pos = [0.49634, -0.06063, 0.03032]
#     end_position = [0.128, -0.368, -0.127]
#     trajectory_duration = 5

#     right_end_effector_start_pos = [-0.49634, -0.04137, 0.03032]
#     right_end_pos = [-0.131, -0.368, -0.127]
#     # Define the values to loop through for [x, y, z]
#     orientation_values = [-1, 0, 1]

#     print("\n--- STARTING ORIENTATION TEST LOOP ---")

#     for x in orientation_values:
#         for y in orientation_values:
#             for z in orientation_values:
                
#                 # Skip the [0, 0, 0] vector as it's invalid (zero length)
#                 if x == 0 and y == 0 and z == 0:
#                     continue
                    
#                 target_orientation = np.array([x, y, z])
                
#                 print(f"\n=======================================================")
#                 print(f"TESTING ORIENTATION: {target_orientation}")
#                 print(f"=======================================================")

#                 # 1. Initialize Robot to zero position
#                 controller.initialize_robot_position()
                
#                 # Pause to observe the reset
#                 time.sleep(2.0) 

#                 # 2. Execute the trajectory
#                 print("Executing trajectory...")
#                 final_ik_left, left_trajectory = controller.execute_arm_trajectory(
#                     arm_side="right",
#                     start_pos=right_end_effector_start_pos,
#                     end_pos=right_end_pos,
#                     duration=trajectory_duration,
#                     target_orientation=target_orientation,
#                     ik_threshold=3.0 # From original 'threshold'
#                 )
                
#                 print(f"--- FINISHED {target_orientation}. Pausing for 3 seconds... ---")
#                 time.sleep(3.0) # Pause to observe the final pose

#     print("\n--- ORIENTATION TEST LOOP COMPLETE ---")