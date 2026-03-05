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
        
        # Add target marker for chess piece placement
        self.scene.add_entity(
            gs.morphs.Sphere(
                radius=0.02,
                pos=(0.0, 0.4, 0.65),
                fixed=True,
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
