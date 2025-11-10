import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import sys, time, math, array
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.Testing.pickAndPlaceClass import KoalbyArmController

class ArmDemo:
    def __init__(self, is_real=False, urdf_path="backend/Testing/Humanoid_URDF-9-10.urdf"):
        print("--- Initializing Controller ---")
        self.controller = KoalbyArmController(is_real=is_real, urdf_path=urdf_path)
    
        self.trajectory_duration = 1
        self.ik_threshold = 3.0
        self.target_orientation = np.array([0, -1, 1])

        self.left_end_effector_start_pos = [0.49634, -0.06063, 0.03032]
        self.right_end_effector_start_pos = [-0.49634, -0.04137, 0.03032]

        self.left_target_1 = [0.128, -0.175, -0.127]
        self.left_target_2 = [0.128, -0.368, -0.127]
        self.right_target_1 = [-0.131, -0.175, -0.127]
        self.right_target_2 = [-0.131, -0.368, -0.127]
        # --------------------------------

        self.state = "WAITING"  # Initial state
        self.pending_target_1 = None
        self.pending_target_2 = None

        # --- Robot Initialization ---
        self.controller.initialize_robot_position()

        #Set the initial IK guess to the defined start positions
        print("Setting initial IK guesses to start positions...")
        try:
            self.controller.ik_solution_left = self.controller.left_arm_chain.inverse_kinematics(
                self.left_end_effector_start_pos,
                initial_position=self.controller.INITIAL_ARM_ANGLES,
                target_orientation=self.target_orientation,
                orientation_mode="Y"
            )
            self.controller.ik_solution_right = self.controller.right_arm_chain.inverse_kinematics(
                self.right_end_effector_start_pos,
                initial_position=self.controller.INITIAL_ARM_ANGLES,
                target_orientation=self.target_orientation,
                orientation_mode="Y"
            )

            self.home_ik_solution_left = list(self.controller.ik_solution_left)
            self.home_ik_solution_right = list(self.controller.ik_solution_right)
            # -----------------------------------------------

            print("Initial IK guesses updated and home IK solutions stored. Robot is in WAITING state.")
        except Exception as e:
            print(f"Error during initial IK calculation: {e}")
            print("Please check URDF path and start positions.")
            sys.exit(1)

    def trigger_move_sequence(self, target_pos_1, target_pos_2):
        """Triggers the state machine to perform a move sequence."""
        if self.state == "WAITING":
            self.pending_target_1 = target_pos_1
            self.pending_target_2 = target_pos_2
            self.state = "MOVE_PIECE"
            print(f"Trigger received. State changed to MOVE_PIECE.")
        else:
            print(f"Cannot start move. Robot is busy in state: {self.state}")

    def update(self):
        if self.state == "WAITING":
            # Robot is idle, do nothing
            time.sleep(0.1)
            pass

        elif self.state == "MOVE_PIECE":
            print(f"--- Executing MOVE_PIECE state ---")
            
            if self.pending_target_1 is not None and self.pending_target_2 is not None:
                
                self.move_to_position_with_preferred_arm(
                    self.pending_target_1,
                    self.pending_target_2
                )
                
                self.pending_target_1 = None
                self.pending_target_2 = None
            else:
                print("Error: MOVE_PIECE state entered but no targets were set.")
            
            print(f"--- MOVE_PIECE state complete. Returning to WAITING state ---")
            self.state = "WAITING"

    def perform_arm_sequence(self, arm_side, target_pos_1, target_pos_2):
        print(f"\n=== STARTING {arm_side.upper()} ARM SEQUENCE ===")

        #Get the defined home position and orientation for this arm
        if arm_side == "left":
            target_orientation = self.target_orientation
            home_pos = self.left_end_effector_start_pos 
            current_fk = self.controller.left_arm_chain.forward_kinematics(self.controller.ik_solution_left)
        elif arm_side == "right":
            target_orientation = self.target_orientation
            home_pos = self.right_end_effector_start_pos
            current_fk = self.controller.right_arm_chain.forward_kinematics(self.controller.ik_solution_right)
        else:
            raise ValueError("arm_side must be 'left' or 'right'")

        intermediate_pos_1 = [target_pos_1[0], target_pos_1[1], target_pos_1[2] + 0.05]
        intermediate_pos_2 = [target_pos_2[0], target_pos_2[1], target_pos_2[2] + 0.05]

        current_pos_at_start = current_fk[:3, 3]

        print("Step 1: Moving to Intermediate 1")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=current_pos_at_start, # Start from where the arm actually is
            end_pos=intermediate_pos_1,
            duration=self.trajectory_duration,
            target_orientation=target_orientation,
            ik_threshold=self.ik_threshold
        )
        
        print("Step 2: Moving to Target 1")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=intermediate_pos_1,
            end_pos=target_pos_1,
            duration=self.trajectory_duration,
            target_orientation=target_orientation,
            ik_threshold=self.ik_threshold
        )

        print("Step 3: Lifting from Target 1")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=target_pos_1,
            end_pos=intermediate_pos_1,
            duration=self.trajectory_duration,
            target_orientation=target_orientation,
            ik_threshold=self.ik_threshold
        )

        print("Step 4: Moving to Intermediate 2")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=intermediate_pos_1,
            end_pos=intermediate_pos_2,
            duration=self.trajectory_duration,
            target_orientation=target_orientation,
            ik_threshold=self.ik_threshold
        )

        print("Step 5: Moving to Target 2")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=intermediate_pos_2,
            end_pos=target_pos_2,
            duration=self.trajectory_duration,
            target_orientation=target_orientation,
            ik_threshold=self.ik_threshold
        )

        print("Step 6: Lifting from Target 2")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=target_pos_2,
            end_pos=intermediate_pos_2,
            duration=self.trajectory_duration,
            target_orientation=target_orientation,
            ik_threshold=self.ik_threshold
        )

        print("Step 7: Returning to Home Position (Smoothly)")
        _, _ = self.controller.execute_arm_trajectory(
            arm_side=arm_side,
            start_pos=intermediate_pos_2,
            end_pos=home_pos, # <-- Use the defined home_pos, not 'start_pos'
            duration=self.trajectory_duration,
            target_orientation=target_orientation, # Use the defined home orientation
            ik_threshold=self.ik_threshold
        )

        print(f"Step 8: Resynchronizing {arm_side} arm state to 'home' IK solution.")
        if arm_side == "left":
            self.controller.ik_solution_left = self.home_ik_solution_left
        else:
            self.controller.ik_solution_right = self.home_ik_solution_right

        print(f"=== FINISHED {arm_side.upper()} ARM SEQUENCE ===\n")

    def move_to_position_with_preferred_arm(self, target_pos_1, target_pos_2):
        print(f"\n--- Calculating Preferred Arm for Sequence ({target_pos_1} -> {target_pos_2}) ---")

        # 1. Get current end-effector positions for both arms
        current_fk_left = self.controller.left_arm_chain.forward_kinematics(self.controller.ik_solution_left)
        current_pos_left = current_fk_left[:3, 3]
        
        current_fk_right = self.controller.right_arm_chain.forward_kinematics(self.controller.ik_solution_right)
        current_pos_right = current_fk_right[:3, 3]

        print(f"Current Left Arm Position: {current_pos_left}")
        print(f"Current Right Arm Position: {current_pos_right}")

        dist_left = np.linalg.norm(np.array(target_pos_1) - current_pos_left)
        dist_right = np.linalg.norm(np.array(target_pos_1) - current_pos_right)

        print(f"Distance to Left: {dist_left:.4f}")
        print(f"Distance to Right: {dist_right:.4f}")

        if dist_left <= dist_right:
            print("Preferred Arm: LEFT")
            preferred_arm = "left"
            start_pos = current_pos_left
        else:
            print("Preferred Arm: RIGHT")
            preferred_arm = "right"
            start_pos = current_pos_right

        print(f"Executing full sequence for {preferred_arm} arm...")
        self.perform_arm_sequence(
            arm_side=preferred_arm,
            target_pos_1=target_pos_1,
            target_pos_2=target_pos_2
        )
        print(f"--- Preferred Arm Sequence Complete ---")

if __name__ == "__main__":
    is_real_robot = False
    urdf_path = "backend/Testing/Humanoid_URDF_9-10.urdf" 

    try:
        demo = ArmDemo(is_real=is_real_robot, urdf_path=urdf_path)
    except FileNotFoundError:
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred during initialization: {e}")
        sys.exit(1)

    print("\n--- Arm Demo State Machine Initialized ---")
    
    targets_queue = [
        # Right-side targets
        [-0.131, -0.368, -0.127],
        [-0.095, -0.368, -0.127], 
        [-0.058, -0.368, -0.127], 
        [-0.021, -0.368, -0.127],
        # Left-side targets
        [0.015, -0.368, -0.127],
        [0.052, -0.368, -0.127],
        [0.090, -0.368, -0.127],
        [0.128, -0.368, -0.127]
    ]
    
    print(f"Ready to process {len(targets_queue)} moves.")

    try:
        while True:
            demo.update()

            if demo.state == "WAITING":   
                
                if targets_queue:
                    next_target = targets_queue.pop(0) 
                    if next_target[0] >= 0:
                        start_pick_pos = demo.left_target_1
                    else:
                        start_pick_pos = demo.right_target_1
                    # ---------------------------------
                    print(f"Triggering move sequence: {start_pick_pos} -> {next_target}")
                    
                    demo.trigger_move_sequence(start_pick_pos, next_target)
                
                else:
                    print("\nMove queue is empty. Demo complete.")
                    break 

    except KeyboardInterrupt:
        print("\nDemo interrupted. Exiting.")
    except Exception as e:
        print(f"\nAn error occurred during the demo: {e}")