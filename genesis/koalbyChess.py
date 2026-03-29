import genesis as gs
import numpy as np
import torch
from koalbyArmController import RobotArmController


class KoalbyChessArmController(RobotArmController):
    """Subclass for controlling the Koalby robot arm specifically for chess tasks."""
    
    def _setup_scene(self):
        """Initialize the Genesis scene with custom setup for chess tasks."""
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(3.5, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
                max_FPS=60,
            ),
            rigid_options = gs.options.RigidOptions(
                enable_neutral_collision=True
            )
        )
        
        # Add ground plane
        self.scene.add_entity(gs.morphs.Plane())
        
        # Add robot
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=self.urdf_file,
                pos=self.robot_pos,
                quat=(0, 0, 0, 1),
                fixed=True,
            )
        )
        
        # Add chess board (custom for chess tasks)
        self.scene.add_entity(
            gs.morphs.Box(
                size=(0.4, 0.5, 0.05),
                pos=(0.0, 0.45, 0.60),
                collision=True,
                fixed=True,
            )
        )
        
        # Add demo chess piece
        piece_size = (0.015, 0.015, 0.03) # Small rectangular prism for piece
        piece_z = 0.615  # Bottom of piece rests on board surface
        piece_x = 0.0  # Center of board
        
        self.scene.add_entity(
            gs.morphs.Box(
                size=piece_size,
                pos=(piece_x, 0.3, piece_z),
                collision=True,
            )
        )
        
        self.scene.build()
    
    def move_to_ready_position(self):
        """Move both grippers to specified ready positions using IK."""
        print("\nMoving to ready position...")
        
        try:
            # Right gripper target position
            right_midpoint = np.array([0.6, 0.0, 0.75])
            right_target = np.array([0.4, 0.5, 0.75])
            print(f"Moving right gripper to {right_target}")
            
            self.ee_name = "gripper_right"
            self._setup_arm_joints()

            ik_right = self.move_to_ik_target(right_midpoint)
            path_right = self.plan_trajectory(ik_right, num_waypoints=50)
            self.execute_trajectory(path_right, pause_steps=50)

            ik_right = self.move_to_ik_target(right_target)
            path_right = self.plan_trajectory(ik_right, num_waypoints=50)
            self.execute_trajectory(path_right, pause_steps=50)
            
            # Left gripper target position
            left_midpoint = np.array([-0.6, 0.0, 0.75])
            left_target = np.array([-0.4, 0.5, 0.75])
            print(f"Moving left gripper to {left_target}")
            
            self.ee_name = "gripper_left"
            self._setup_arm_joints()

            ik_left = self.move_to_ik_target(left_midpoint)
            path_left = self.plan_trajectory(ik_left, num_waypoints=50)
            self.execute_trajectory(path_left, pause_steps=50)
            
            ik_left = self.move_to_ik_target(left_target)
            path_left = self.plan_trajectory(ik_left, num_waypoints=50)
            self.execute_trajectory(path_left, pause_steps=50)
            
            # Reset to right gripper as default
            self.ee_name = "gripper_right"
            self._setup_arm_joints()
            
            print("Ready position achieved!")
            
        except Exception as e:
            print(f"Error moving to ready position: {e}")
    
    def pick_and_place(self, source_pos: np.ndarray, dest_pos: np.ndarray, 
                       gripper_name: str = "gripper_right", 
                       above_offset: float = 0.15,
                       target_quat: np.ndarray = None,
                       num_waypoints: int = 50):
        """Pick up a chess piece from source position and place it at destination.
        
        Args:
            source_pos: Source position [x, y, z] of the chess piece
            dest_pos: Destination position [x, y, z] where to place the piece
            gripper_name: Which gripper to use ("gripper_left" or "gripper_right")
            above_offset: Height above piece to approach from (default: 0.1m)
            target_quat: Target quaternion for gripper orientation (default: None = identity)
            num_waypoints: Number of waypoints for trajectories (default: 50)
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Set the end effector
            self.ee_name = gripper_name
            self._setup_arm_joints()
            
            print(f"\n========== Pick and Place Operation ==========")
            print(f"Source: {source_pos}")
            print(f"Destination: {dest_pos}")
            print(f"Using: {gripper_name}")
            
            # Step 1: Move above source position with target orientation
            above_source = source_pos.copy()
            above_source[2] += above_offset
            
            print(f"\n1. Moving above source position with target orientation...")
            target_orientation = target_quat if target_quat is not None else np.array([0, 0, 0, 1])
            ik_above = self.move_to_ik_target(above_source, target_quat=target_orientation)
            if ik_above is None:
                print("Failed to compute IK for above source position")
                return False
            
            path = self.plan_trajectory(ik_above, num_waypoints=num_waypoints)
            self.execute_trajectory(path, pause_steps=50)
            
            # Step 2: Open gripper
            print(f"\n2. Opening gripper...")
            self.open_gripper(force=0.5, duration_steps=30)
            
            # Step 3: Move down to source position
            print(f"\n3. Moving down to chess piece...")
            ik_source = self.move_to_ik_target(source_pos, target_quat=target_orientation)
            if ik_source is None:
                print("Failed to compute IK for source position")
                return False
            
            path = self.plan_trajectory(ik_source, num_waypoints=num_waypoints)
            self.execute_trajectory(path, pause_steps=30)
            
            # Step 4: Close gripper (grab piece)
            print(f"\n4. Closing gripper to grab piece...")
            self.close_gripper(force=-0.5, duration_steps=30)
            
            # Step 5: Move back up
            print(f"\n5. Moving back up with piece...")
            path = self.plan_trajectory(ik_above, num_waypoints=num_waypoints)
            self.execute_trajectory(path, pause_steps=50)
            
            # Step 6: Move to destination
            print(f"\n6. Moving to destination position...")
            above_dest = dest_pos.copy()
            above_dest[2] += above_offset
            
            ik_above_dest = self.move_to_ik_target(above_dest)
            if ik_above_dest is None:
                print("Failed to compute IK for above destination position")
                return False
            
            path = self.plan_trajectory(ik_above_dest, num_waypoints=num_waypoints)
            self.execute_trajectory(path, pause_steps=50)
            
            # Step 7: Move down to destination
            print(f"\n7. Moving down to destination...")
            ik_dest = self.move_to_ik_target(dest_pos)
            if ik_dest is None:
                print("Failed to compute IK for destination position")
                return False
            
            path = self.plan_trajectory(ik_dest, num_waypoints=num_waypoints)
            self.execute_trajectory(path, pause_steps=30)
            
            # Step 8: Open gripper to place piece
            print(f"\n8. Opening gripper to place piece...")
            self.open_gripper(force=0.5, duration_steps=30)
            
            # Step 9: Move back up
            print(f"\n9. Moving back up...")
            path = self.plan_trajectory(ik_above_dest, num_waypoints=num_waypoints)
            self.execute_trajectory(path, pause_steps=50)
            
            print(f"\n========== Pick and Place Complete! ==========\n")
            return True
            
        except Exception as e:
            print(f"Error during pick and place: {e}")
            return False


# -------------------------
# Main Execution
# -------------------------
if __name__ == "__main__":
    controller = KoalbyChessArmController(
        urdf_file="Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf",
        ee_name="gripper_right",
        robot_pos=(0.0, 0.0, 0.735),
        show_viewer=True,
        cache_file="chess_paths.pt",
    )
    
    # Move to ready position before accepting commands
    controller.move_to_ready_position()
    
    # Perform pick and place operation
    print("\n" + "="*50)
    print("Performing demo pick and place operation...")
    print("="*50)
    
    source_position = np.array([0.0, 0.3, 0.615])  # Demo piece position
    dest_position = np.array([0.15, 0.45, 0.65])   # Move to right side of board
    
    controller.pick_and_place(
        source_pos=source_position,
        dest_pos=dest_position,
        gripper_name="gripper_left",
        above_offset=0.15,
        num_waypoints=50
    )
    
    controller.run_interactive_loop()
