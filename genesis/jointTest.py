import genesis as gs
import numpy as np

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
        quat=(1, 0, 0, 0),
        fixed=True,
    )
)

scene.build()

# -------------------------
# Get controllable joints
# -------------------------
active_joints = [j for j in finley.joints if j.n_dofs > 0]

print("\n--- Controllable Joints ---")
for i, joint in enumerate(active_joints):
    print(f"[{i}] {joint.name}")
print("--------------------------")

# -------------------------
# Choose joint
# -------------------------
joint_idx = int(input("Select joint index: "))
joint = active_joints[joint_idx]
dof_idx = joint.dof_idx_local

print(f"\nControlling joint: {joint.name}")

# -------------------------
# Control loop
# -------------------------
STEP = 0.1
qpos = finley.get_qpos().clone()

while True:
    key = input("Input 'q' to quit, 'c' to change motors, or how much you want the motor to spin (in rads): ")

    if key == "q":
        break

    elif key == "c":
        active_joints = [j for j in finley.joints if j.n_dofs > 0]

        print("\n--- Controllable Joints ---")
        for i, joint in enumerate(active_joints):
            print(f"[{i}] {joint.name}")
        print("--------------------------")

        joint_idx = int(input("Select joint index: "))
        joint = active_joints[joint_idx]
        dof_idx = joint.dofs_idx_local

        print(f"\nControlling joint: {joint.name}")

    else:
        qpos[dof_idx] = float(key)

        finley.control_dofs_position(qpos)

        # step a few frames so motion is visible
        for _ in range(20):
            scene.step()
