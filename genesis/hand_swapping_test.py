import genesis as gs
import numpy as np
import torch
from koalbyArmController import RobotArmController


class HandSwappingTest(RobotArmController):
    """Dual-arm swapping test harness using the base RobotArmController."""

    def _setup_scene(self):
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(3.5, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
                max_FPS=60,
            ),
            rigid_options=gs.options.RigidOptions(enable_neutral_collision=True),
        )

        self.scene.add_entity(gs.morphs.Plane())

        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=self.urdf_file,
                pos=self.robot_pos,
                quat=(0, 0, 0, 1),
                fixed=True,
            )
        )

        # Optional swapping station geometry
        self.scene.add_entity(
            gs.morphs.Box(
                size=(0.6, 0.6, 0.05),
                pos=(0.0, 0.4, 0.62),
                collision=True,
                fixed=True,
            )
        )

        # Dual test pieces
        self.scene.add_entity(gs.morphs.Box(size=(0.02, 0.02, 0.04), pos=(0.15, 0.4, 0.64), collision=True))
        self.scene.add_entity(gs.morphs.Box(size=(0.02, 0.02, 0.04), pos=(-0.15, 0.4, 0.64), collision=True))

        self.scene.build()

    def move_arm_ready(self):
        right_xy = np.array([0.45, 0.5, 0.75])
        left_xy = np.array([-0.45, 0.5, 0.75])

        for ee_name, target in [("wrist_right", right_xy), ("wrist_left", left_xy)]:
            self.ee_name = ee_name
            self._setup_arm_joints()
            ik = self.move_to_ik_target(target)
            if ik is None:
                raise RuntimeError(f"IK failed for {ee_name} at {target}")
            self.execute_trajectory(self.plan_trajectory(ik, num_waypoints=60), pause_steps=40)

        self.ee_name = "wrist_right"
        self._setup_arm_joints()

    def pick(self, position, ee_name, above_offset=0.12, num_waypoints=45):
        self.ee_name = ee_name
        self._setup_arm_joints()

        above = position.copy(); above[2] += above_offset
        ik_above = self.move_to_ik_target(above)
        if ik_above is None:
            return False

        self.execute_trajectory(self.plan_trajectory(ik_above, num_waypoints=num_waypoints), pause_steps=20)
        self.open_gripper(force=0.6, duration_steps=30)

        ik_down = self.move_to_ik_target(position)
        if ik_down is None:
            return False

        self.execute_trajectory(self.plan_trajectory(ik_down, num_waypoints=num_waypoints), pause_steps=20)
        self.close_gripper(force=-0.6, duration_steps=30)

        self.execute_trajectory(self.plan_trajectory(ik_above, num_waypoints=num_waypoints), pause_steps=20)
        return True

    def place(self, position, ee_name, above_offset=0.12, num_waypoints=45):
        self.ee_name = ee_name
        self._setup_arm_joints()

        above = position.copy(); above[2] += above_offset
        ik_above = self.move_to_ik_target(above)
        if ik_above is None:
            return False

        self.execute_trajectory(self.plan_trajectory(ik_above, num_waypoints=num_waypoints), pause_steps=20)

        ik_down = self.move_to_ik_target(position)
        if ik_down is None:
            return False

        self.execute_trajectory(self.plan_trajectory(ik_down, num_waypoints=num_waypoints), pause_steps=20)
        self.open_gripper(force=0.6, duration_steps=30)

        self.execute_trajectory(self.plan_trajectory(ik_above, num_waypoints=num_waypoints), pause_steps=20)
        return True

    def swap(self, pos_a, pos_b, above_offset=0.12, num_waypoints=45):
        self.move_arm_ready()

        if not self.pick(pos_a, ee_name="wrist_right", above_offset=above_offset, num_waypoints=num_waypoints):
            print("Failed to pick piece A with right gripper")
            return False

        if not self.pick(pos_b, ee_name="wrist_left", above_offset=above_offset, num_waypoints=num_waypoints):
            print("Failed to pick piece B with left gripper")
            return False

        if not self.place(pos_a, ee_name="wrist_left", above_offset=above_offset, num_waypoints=num_waypoints):
            print("Failed to place piece B into A position")
            return False

        if not self.place(pos_b, ee_name="wrist_right", above_offset=above_offset, num_waypoints=num_waypoints):
            print("Failed to place piece A into B position")
            return False

        self.move_arm_ready()
        return True


if __name__ == "__main__":
    controller = HandSwappingTest(
        urdf_file="SwappingURDF//urdf//SwappingURDF.urdf",
        ee_name="wrist_right",
        robot_pos=(0.0, 0.0, 0.735),
        show_viewer=True,
        cache_file="hand_swap_cache.pt",
    )

    controller.move_arm_ready()

    pos_a = np.array([0.15, 0.35, 0.615])
    pos_b = np.array([-0.15, 0.35, 0.615])

    success = controller.swap(pos_a, pos_b, above_offset=0.15, num_waypoints=60)
    print("Hand swap test", "passed" if success else "failed")

    controller.run_interactive_loop()
