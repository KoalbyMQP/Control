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
        collision=True,
    )
)

cart = scene.add_entity(
    gs.morphs.Box(
        size=(0.3, 0.6, 0.6),
        pos=(0.0, 0.5, 0.3),
        collision=True,
    )
)

target_marker = scene.add_entity(
        gs.morphs.Sphere(
            radius=0.02,
            pos=(0.0,0.4,0.65),
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

cached_paths = torch.load('test_path.pt')


# -------------------------------------------------
# Choose End Effector Link Name
# -------------------------------------------------
EE_NAME = "gripper_right"  

ee_link = finley.get_link(EE_NAME)

if ee_link is None:
    raise ValueError(f"End effector '{EE_NAME}' not found!")

# -------------------------------------------------
# Convert joint names -> dofs_idx_local
# -------------------------------------------------
arm_dofs_idx_local = []

if EE_NAME == "gripper_left":
    offset = 0.145
    for joint in finley.joints:
        if joint.name in left_arm_joints:
            if joint.n_dofs > 0:
                arm_dofs_idx_local.extend(joint.dofs_idx_local)

elif EE_NAME == "gripper_right":
    offset = -0.145
    for joint in finley.joints:
        if joint.name in right_arm_joints:
            if joint.n_dofs > 0:
                arm_dofs_idx_local.extend(joint.dofs_idx_local)

else:
    raise ValueError("No gripper specified")

print("\nArm DOFs local indices:", arm_dofs_idx_local)

# -------------------------------------------------
# Control Loop
# -------------------------------------------------
while True:

    cmd = input("\n'q' to quit, 'ik' to move arm with IK, 'r' to choose previous position, 'c' to clear cached paths: ")
    if cmd == "c":
        cached_paths = {}
        key_counter = 0
        print("Cached paths cleared.")
        continue

    if cmd == "q":
        torch.save(cached_paths, 'test_path.pt')
        break

    if cmd == "ik":
        save_flag = False
        x = float(input("Target X: "))
        y = float(input("Target Y: "))
        z = float(input("Target Z: "))

        if save_flag:
            save_answer = input("Save this path? (y/n): ").lower()
            if save_answer == 'y':
                save_flag = True
                key_counter = str(input("Path Key: "))
                print("This path will be saved.")
            else:
                ("This path will NOT be saved.")

        target_pos = torch.tensor([x + offset, y, z], dtype=torch.float32)

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
            num_waypoints = 200,
        )

        if save_flag:
            cached_paths[key_counter] = path
        
        print(path)

        print("Executing trajectory...")

        for waypoint in path:
            finley.control_dofs_position(
                waypoint,
            )
            scene.step()

        for _ in range(100):
            scene.step()
    if cmd == "r":
        for key in cached_paths.keys():
            print(f"Key: {key}")

        path = cached_paths[str(input("Path Key: "))]

        print(path)

        print("Executing trajectory...")

        for waypoint in path:
            finley.control_dofs_position(
                waypoint,
            )
            scene.step()

        for _ in range(100):
            scene.step()