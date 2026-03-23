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
                pos=(0.0, 0.45, 0.55),
                collision=True,
                fixed=True,
            )
        )
        
        # Add demo chess piece
        piece_size = (0.015, 0.015, 0.03) # Small rectangular prism for piece
        piece_z = 0.59  # Bottom of piece rests on board surface
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
            right_midpoint = np.array([0.4, 0.0, 0.65])
            right_target = np.array([0.4, 0.5, 0.65])
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
            left_midpoint = np.array([-0.4, 0.0, 0.65])
            left_target = np.array([-0.4, 0.5, 0.65])
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
    
    controller.run_interactive_loop()
