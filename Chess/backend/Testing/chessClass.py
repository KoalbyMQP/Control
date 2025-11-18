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

    def trigger_move_sequence(self, target_pos_1, target_pos_2, intermediate_pos=None):
        """Triggers the state machine to perform a move sequence."""
        if self.state == "WAITING":
            self.pending_target_1 = target_pos_1
            self.pending_target_2 = target_pos_2
            self.pending_intermediate = intermediate_pos
            self.state = "MOVE_PIECE"
            print(f"Trigger received. State changed to MOVE_PIECE.")
        else:
            print(f"Cannot start move. Robot is busy in state: {self.state}")

    def trigger_capture_sequence(self, moving_piece_pos, captured_piece_pos, discard_pos, intermediate_pos=None):
        """Triggers the state machine to perform a capture sequence."""
        if self.state == "WAITING":
            # Store all positions needed for the multi-step operation
            self.pending_moving_piece_pos = moving_piece_pos
            self.pending_captured_piece_pos = captured_piece_pos
            self.pending_discard_pos = discard_pos
            self.pending_intermediate = intermediate_pos    
            
            # Clear other pending vars
            self.pending_target_1 = None
            self.pending_target_2 = None
            
            self.state = "CAPTURE_PIECE"
            print(f"Trigger received. State changed to CAPTURE_PIECE.")
        else:
            print(f"Cannot start capture. Robot is busy in state: {self.state}")

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
                    self.pending_target_2,
                    self.pending_intermediate
                )
                
                self.pending_target_1 = None
                self.pending_target_2 = None
                self.pending_intermediate = None
            else:
                print("Error: MOVE_PIECE state entered but no targets were set.")
            
            print(f"--- MOVE_PIECE state complete. Returning to WAITING state ---")
            self.state = "WAITING"

        elif self.state == "CAPTURE_PIECE":
            print(f"--- Executing CAPTURE_PIECE (Step 1: Remove Piece) ---")
            
            if self.pending_captured_piece_pos is not None and self.pending_discard_pos is not None:
                # Step 1: Move the captured piece to the discard pile
                print(f"Capturing piece at {self.pending_captured_piece_pos} and moving to discard {self.pending_discard_pos}")
                self.move_to_position_with_preferred_arm(
                    self.pending_captured_piece_pos,
                    self.pending_discard_pos,
                    self.pending_intermediate
                )
                
                print(f"--- CAPTURE_PIECE (Step 1) complete. ---")
                
                # Now, set up for Step 2: Move the attacking piece
                print(f"--- Setting up CAPTURE_PIECE (Step 2: Move Attacker) ---")
                
                # Set the pending vars for the MOVE_PIECE state
                self.pending_target_1 = self.pending_moving_piece_pos
                self.pending_target_2 = self.pending_captured_piece_pos
                # self.pending_intermediate is already set and will be passed along
                
                # Clear the capture-specific vars
                self.pending_moving_piece_pos = None
                self.pending_captured_piece_pos = None
                self.pending_discard_pos = None
                
                # CRITICAL: Transition state to MOVE_PIECE to execute Step 2
                self.state = "MOVE_PIECE" 

            else:
                print("Error: CAPTURE_PIECE state entered but no targets were set.")
                self.state = "WAITING" # Bail out to waiting
            
            # Note: We do NOT set state to WAITING here. We chain to MOVE_PIECE.

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

    def move_to_position_with_preferred_arm(self, target_pos_1, target_pos_2, intermediate_pos=None):
        print(f"\n--- Analyzing Move: {target_pos_1} -> {target_pos_2} ---")

        # 1. Define Coordinate Logic
        # Assuming X > 0 is Left Side, X < 0 is Right Side (Based on your target queue)
        # Assuming X = 0 is the center line
        start_x = target_pos_1[0]
        end_x = target_pos_2[0]
        
        # Handoff point logic: Use passed value or fallback to default
        if intermediate_pos is not None:
            handoff_pos = intermediate_pos
            print(f"Using Provided Intermediate Position: {handoff_pos}")
        else:
            # Default fallback if none provided
            handoff_pos = [0.0, -0.3, target_pos_1[2]] 
            print(f"No Intermediate provided. Using calculated default: {handoff_pos}")

        is_start_left = start_x >= 0
        is_end_left = end_x >= 0

        # 2. Check for Cross-Board Movement
        if is_start_left != is_end_left:
            print(">>> CROSS-BOARD MOVE DETECTED (Intersection Prevention Active) <<<")
            
            # CASE A: Left to Right
            if is_start_left:
                print("Sequence: Left Arm -> Handoff -> Right Arm")
                
                # Step A: Left Arm moves Piece to Center
                self.perform_arm_sequence(
                    arm_side="left",
                    target_pos_1=target_pos_1,
                    target_pos_2=handoff_pos
                )
                
                print(">>> Handoff Point Reached. Switching Arms. <<<")
                
                # Step B: Right Arm moves Piece from Center to Destination
                self.perform_arm_sequence(
                    arm_side="right",
                    target_pos_1=handoff_pos,
                    target_pos_2=target_pos_2
                )

            # CASE B: Right to Left
            else:
                print("Sequence: Right Arm -> Handoff -> Left Arm")
                
                # Step A: Right Arm moves Piece to Center
                self.perform_arm_sequence(
                    arm_side="right",
                    target_pos_1=target_pos_1,
                    target_pos_2=handoff_pos
                )

                print(">>> Handoff Point Reached. Switching Arms. <<<")

                # Step B: Left Arm moves Piece from Center to Destination
                self.perform_arm_sequence(
                    arm_side="left",
                    target_pos_1=handoff_pos,
                    target_pos_2=target_pos_2
                )

        # 3. Standard Single-Arm Movement (No Crossing)
        else:
            # Determine arm based on side (Left side uses Left arm, Right uses Right)
            # This is safer than distance calculation if we have strictly defined zones
            if is_start_left:
                preferred_arm = "left"
            else:
                preferred_arm = "right"

            print(f"Standard Move (Same Side): Using {preferred_arm.upper()} arm.")
            
            self.perform_arm_sequence(
                arm_side=preferred_arm,
                target_pos_1=target_pos_1,
                target_pos_2=target_pos_2
            )
        
        print(f"--- Move Sequence Complete ---")

if __name__ == "__main__":
    is_real_robot = False
    urdf_path = "backend/Testing/Humanoid_URDF_9-10.urdf" 

    try:
        # Ensure ArmDemo is available (Assuming this file is pasted into the main project)
        # If running standalone, this part would require the 'ArmDemo' class definition.
        # We assume 'ArmDemo' inherits from 'ArmDemoLogic' in your actual setup.
        demo = ArmDemo(is_real=is_real_robot, urdf_path=urdf_path)
    except FileNotFoundError:
        sys.exit(1)
    except NameError:
        print("Error: 'ArmDemo' class not found. Ensure this logic is mixed into your main class.")
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred during initialization: {e}")
        sys.exit(1)

    print("\n--- Arm Demo State Machine Initialized ---")
    
    # Define safety defaults for picking positions if not already set
    if not hasattr(demo, 'left_target_1'):
        print("Setting default Left Pick Position...")
        demo.left_target_1 = [0.20, -0.368, -0.127] 
    
    if not hasattr(demo, 'right_target_1'):
        print("Setting default Right Pick Position...")
        demo.right_target_1 = [-0.20, -0.368, -0.127]

    # --- Test Setup ---
    # Define the external intermediate point (e.g. Center of board)
    global_intermediate_point = [0.0, -0.3, -0.127]
    # Define a discard point for captured pieces
    global_discard_point = [0.25, -0.2, -0.1] # e.g., a "captured pieces" box on the left

    # Define a queue of actions to perform:
    # (ACTION_TYPE, pos_A, pos_B, pos_C)
    action_queue = [
        # 1. Test a standard move (cross-board)
        # ("MOVE", start, end, None)
        # ("MOVE", demo.left_target_1, [-0.131, -0.368, -0.127], None),
        
        # 2. Test a capture (cross-board)
        # ("CAPTURE", moving_piece_start, captured_piece_pos, discard_pos)
        ("CAPTURE", demo.left_target_1, [-0.095, -0.368, -0.127], global_discard_point),
        
        # 3. Test another standard move (same-side)
        # ("MOVE", [0.090, -0.368, -0.127], [0.128, -0.368, -0.127], None),
    ]
    

    print(f"Ready to process {len(action_queue)} actions.")

    try:
        while True:
            demo.update()

            if demo.state == "WAITING":   
                
                if action_queue:
                    action = action_queue.pop(0) 
                    action_type = action[0]
                    
                    if action_type == "MOVE":
                        pos_1 = action[1]
                        pos_2 = action[2]
                        print(f"\n>>> ACTION: MOVE | {pos_1} -> {pos_2}")
                        demo.trigger_move_sequence(
                            pos_1, 
                            pos_2, 
                            intermediate_pos=global_intermediate_point
                        )
                    elif action_type == "CAPTURE":
                        moving_piece_pos = action[1]
                        captured_piece_pos = action[2]
                        discard_pos = action[3]
                        print(f"\n>>> ACTION: CAPTURE | Attacker at {moving_piece_pos}, Target at {captured_piece_pos}")
                        demo.trigger_capture_sequence(
                            moving_piece_pos, 
                            captured_piece_pos,
                            discard_pos,
                            intermediate_pos=global_intermediate_point
                        )
                
                else:
                    print("\nAction queue is empty. Demo complete.")
                    break 

    except KeyboardInterrupt:
        print("\nDemo interrupted. Exiting.")
    except Exception as e:
        print(f"\nAn error occurred during the demo: {e}")