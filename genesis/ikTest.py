import genesis as gs
import numpy as np
import torch

# -------------------------
# Init Genesis + Scene
# -------------------------
gs.init()

scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        res=(1280, 960),
        camera_pos=(3.5, 0.0, 2.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
        max_FPS=60,
    )
)

scene.add_entity(gs.morphs.Plane())

finley = scene.add_entity(
    gs.morphs.URDF(
        file="Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf",
        pos=(0.0, 0.0, 0.735),
        quat=(0, 0, 0, 1),
        fixed=True,
    )
)

scene.build()

left_arm_joints = [
    "shoulderspin_left",
    "armlift_left",
    "elbowcurl_left",
    "handspin_left",
    "wristcurl_left"
]

right_arm_joints = [
    "shoulderspin_right",
    "armlift_right",
    "elbowcurl_right",
    "handspin_right",
    "wristcurl_right"
]

# -------------------------------------------------
# Choose End Effector Link Name
# -------------------------------------------------
EE_NAME = "hand_left"  

ee_link = finley.get_link(EE_NAME)

if ee_link is None:
    raise ValueError(f"End effector '{EE_NAME}' not found!")

# -------------------------------------------------
# Convert joint names -> dofs_idx_local
# -------------------------------------------------
arm_dofs_idx_local = []

if EE_NAME == "hand_left":
    for joint in finley.joints:
        if joint.name in left_arm_joints:
            if joint.n_dofs > 0:
                arm_dofs_idx_local.extend(joint.dofs_idx_local)

elif EE_NAME == "hand_right":
    for joint in finley.joints:
        if joint.name in right_arm_joints:
            if joint.n_dofs > 0:
                arm_dofs_idx_local.extend(joint.dofs_idx_local)

print("\nArm DOFs local indices:", arm_dofs_idx_local)

# -------------------------------------------------
# Control Loop
# -------------------------------------------------
while True:

    cmd = input("\n'q' to quit, 'ik' to move arm with IK: ")

    if cmd == "q":
        break

    if cmd == "ik":

        x = float(input("Target X: "))
        y = float(input("Target Y: "))
        z = float(input("Target Z: "))

        target_pos = torch.tensor([x, y, z], dtype=torch.float32)

        # -----------------------------------
        # Inverse Kinematics (ARM ONLY)
        # -----------------------------------
        ik_result = finley.inverse_kinematics(
            link = ee_link,
            pos = target_pos,
            quat = np.array([0, 0, 0, 1]),
            dofs_idx_local = arm_dofs_idx_local
        )

        print("IK result:", ik_result)

        # -----------------------------------
        # Path Planning (ARM ONLY)
        # -----------------------------------
        current_qpos = finley.get_qpos()

        path = finley.plan_path(
            qpos_goal = ik_result,
            dofs_idx_local = arm_dofs_idx_local
        )

        print(path)

        print("Executing trajectory...")

        for waypoint in path:
            finley.control_dofs_position(
                waypoint,
            )
            scene.step()

        for _ in range(100):
            scene.step()