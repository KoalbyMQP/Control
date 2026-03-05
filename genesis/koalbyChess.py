import genesis as gs
import numpy as np
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
        
        # Add chess board (custom for chess tasks)
        self.scene.add_entity(
            gs.morphs.Box(
                size=(0.4, 0.55, 0.6),
                pos=(0.0, 0.45, 0.3),
                collision=True,
            )
        )
        
        # Add demo chess pieces
        piece_spacing = 0.3 / 7  # Space 8 pieces across 0.3 width
        piece_size = (0.03, 0.03, 0.06) # Small rectangular prisms for pieces
        piece_z = 0.63  # Bottom of pieces rest on board surface (0.6 + 0.03)
        
        for i in range(8):
            piece_x = -0.15 + i * piece_spacing  # Center at 0.0, span from -0.15 to 0.15
            self.scene.add_entity(
                gs.morphs.Box(
                    size=piece_size,
                    pos=(piece_x, 0.3, piece_z),
                    color=(0, 0, 0),  # Black color for better visibility
                    collision=True,
                )
            )
        
        
        self.scene.build()


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
    
    controller.run_interactive_loop()
