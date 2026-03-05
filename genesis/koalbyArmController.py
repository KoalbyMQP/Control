import genesis as gs
import numpy as np
import torch
from pathlib import Path


class RobotArmController:
    """Base class for controlling robot arm with IK and trajectory planning."""
    
    def __init__(
        self,
        urdf_file: str = "Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf",
        ee_name: str = "gripper_right",
        robot_pos: tuple = (0.0, 0.0, 0.735),
        show_viewer: bool = True,
        cache_file: str = "test_path.pt",
    ):
        """Initialize the robot arm controller.
        
        Args:
            urdf_file: Path to the URDF file
            ee_name: End effector link name ("gripper_left" or "gripper_right")
            robot_pos: Initial position of the robot
            show_viewer: Whether to show the Genesis viewer
            cache_file: Path to cache file for trajectories
        """
        self.urdf_file = urdf_file
        self.ee_name = ee_name
        self.robot_pos = robot_pos
        self.show_viewer = show_viewer
        self.cache_file = cache_file
        
        # Joint definitions
        self.left_arm_joints = [
            "shoulderspin_left",
            "armlift_left",
            "elbowcurl_left",
            "handspin_left",
            "wristcurl_left"
        ]
        
        self.right_arm_joints = [
            "shoulderspin_right",
            "armlift_right",
            "elbowcurl_right",
            "handspin_right",
            "wristcurl_right"
        ]
        
        # Initialize Genesis
        gs.init()
        self.scene = None
        self.robot = None
        self.ee_link = None
        self.arm_dofs_idx_local = []
        self.offset = 0.0
        self.cached_paths = {}
        
        # Setup
        self._setup_scene()
        self._setup_arm_joints()
        self._load_cache()
    
    def _setup_scene(self):
        """Initialize the Genesis scene and add entities."""
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(3.5, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
                max_FPS=60,
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
                collision=True,
            )
        )
        
        self.scene.build()
    
    def _setup_arm_joints(self):
        """Configure arm joints and end effector."""
        # Get end effector link
        self.ee_link = self.robot.get_link(self.ee_name)
        
        if self.ee_link is None:
            raise ValueError(f"End effector '{self.ee_name}' not found!")
        
        # Get joint indices for the selected arm
        if self.ee_name == "gripper_left":
            self.offset = 0.145
            joint_names = self.left_arm_joints
        elif self.ee_name == "gripper_right":
            self.offset = -0.145
            joint_names = self.right_arm_joints
        else:
            raise ValueError("End effector must be 'gripper_left' or 'gripper_right'")
        
        # Extract DOF indices for arm joints
        self.arm_dofs_idx_local = []
        for joint in self.robot.joints:
            if joint.name in joint_names:
                if joint.n_dofs > 0:
                    self.arm_dofs_idx_local.extend(joint.dofs_idx_local)
        
        print(f"\nEnd Effector: {self.ee_name}")
        print(f"Arm DOFs local indices: {self.arm_dofs_idx_local}")
    
    def _load_cache(self):
        """Load cached paths from file if it exists."""
        cache_path = Path(self.cache_file)
        if cache_path.exists():
            self.cached_paths = torch.load(self.cache_file)
            print(f"Loaded {len(self.cached_paths)} cached paths")
        else:
            self.cached_paths = {}
    
    def _save_cache(self):
        """Save cached paths to file."""
        torch.save(self.cached_paths, self.cache_file)
        print(f"Saved {len(self.cached_paths)} paths to cache")
    
    def select_end_effector_by_position(self, target_pos: np.ndarray):
        """Automatically select end effector based on target x-coordinate.
        
        Args:
            target_pos: Target position [x, y, z]
        """
        target_ee = "gripper_right" if target_pos[0] >= 0 else "gripper_left"
        
        if target_ee != self.ee_name:
            print(f"Switching from {self.ee_name} to {target_ee}")
            self.ee_name = target_ee
            self._setup_arm_joints()
        else:
            print(f"Using {self.ee_name}")
    
    def move_to_ik_target(self, target_pos: np.ndarray, target_quat: np.ndarray = None) -> torch.Tensor:
        """Solve inverse kinematics for a target position.
        
        Args:
            target_pos: Target position [x, y, z]
            target_quat: Target quaternion [x, y, z, w] (default: [0, 0, 0, 1])
            
        Returns:
            IK solution (joint angles)
        """
        if target_quat is None:
            target_quat = np.array([0, 0, 0, 1])
        
        # Adjust for arm offset
        adjusted_pos = torch.tensor(
            [target_pos[0] + self.offset, target_pos[1], target_pos[2]],
            dtype=torch.float32
        )
        
        # Solve IK
        ik_result = self.robot.inverse_kinematics(
            link=self.ee_link,
            pos=adjusted_pos,
            quat=target_quat,
            dofs_idx_local=self.arm_dofs_idx_local
        )
        
        print(f"IK result: {ik_result}")
        return ik_result
    
    def plan_trajectory(self, target_qpos: torch.Tensor, num_waypoints: int = 200) -> list:
        """Plan a trajectory to target joint configuration.
        
        Args:
            target_qpos: Target joint configuration
            num_waypoints: Number of waypoints in trajectory
            
        Returns:
            Planned trajectory
        """
        path = self.robot.plan_path(
            qpos_goal=target_qpos,
            num_waypoints=num_waypoints,
        )
        
        print(f"Planned trajectory with {len(path)} waypoints")
        return path
    
    def execute_trajectory(self, path: list, pause_steps: int = 100):
        """Execute a trajectory.
        
        Args:
            path: List of waypoints to execute
            pause_steps: Number of steps to pause after trajectory completes
        """
        print(f"Executing trajectory with {len(path)} waypoints...")
        
        for waypoint in path:
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        
        # Pause at end position
        for _ in range(pause_steps):
            self.scene.step()
        
        print("Trajectory complete")
    
    def run_interactive_loop(self):
        """Run the main interactive control loop."""
        print("\n" + "="*50)
        print("Robot Arm Control - Interactive Loop")
        print("="*50)
        print("Commands:")
        print("  'ik'  - Move arm to position using inverse kinematics")
        print("  'r'   - Execute a cached trajectory")
        print("  'c'   - Clear all cached paths")
        print("  'q'   - Quit")
        print("="*50 + "\n")
        
        while True:
            cmd = input("\nEnter command (ik/r/c/q): ").strip().lower()
            
            if cmd == "q":
                self._save_cache()
                print("Exiting...")
                break
            
            elif cmd == "c":
                self.cached_paths = {}
                print("Cached paths cleared.")
            
            elif cmd == "ik":
                self._handle_ik_command()
            
            elif cmd == "r":
                self._handle_replay_command()
            
            else:
                print("Unknown command. Try 'ik', 'r', 'c', or 'q'")
    
    def _handle_ik_command(self):
        """Handle interactive IK command."""
        try:
            x = float(input("Target X: "))
            y = float(input("Target Y: "))
            z = float(input("Target Z: "))
            
            target_pos = np.array([x, y, z])
            
            # Automatically select end effector based on target x-coordinate
            self.select_end_effector_by_position(target_pos)
            
            # Solve IK
            ik_result = self.move_to_ik_target(target_pos)
            
            # Plan and execute trajectory
            path = self.plan_trajectory(ik_result)
            
            # Ask to save
            save_answer = input("Save this path? (y/n): ").strip().lower()
            if save_answer == 'y':
                key = str(input("Path key name: "))
                self.cached_paths[key] = path
                print(f"Path saved as '{key}'")
            
            # Execute
            self.execute_trajectory(path)
            
        except ValueError:
            print("Invalid input. Please enter valid numbers.")
        except Exception as e:
            print(f"Error during IK: {e}")
    
    def _handle_replay_command(self):
        """Handle interactive replay command."""
        if not self.cached_paths:
            print("No cached paths available.")
            return
        
        print("\nAvailable paths:")
        for key in self.cached_paths.keys():
            print(f"  - {key}")
        
        key = str(input("Path key to replay: ")).strip()
        
        if key not in self.cached_paths:
            print(f"Path '{key}' not found.")
            return
        
        path = self.cached_paths[key]
        self.execute_trajectory(path)


# -------------------------
# Main Execution
# -------------------------
if __name__ == "__main__":
    controller = RobotArmController(
        urdf_file="Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf",
        ee_name="gripper_right",
        robot_pos=(0.0, 0.0, 0.735),
        show_viewer=True,
        cache_file="test_path.pt",
    )
    
    controller.run_interactive_loop()
