import genesis as gs
import numpy as np

# -------------------------
# Init Genesis + Scene
# -------------------------
gs.init()

DEBUG = True

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
# Control loop
# -------------------------
qpos = finley.get_qpos().clone()

while True:
    
    key = input("Input 'q' to quit, 'm' to make the motion, or which motor you would like to move (index): ")

    if key == "q":
        break

    elif key.isdigit():
        dof_idx = int(key)

        pos = float(input("How much would you like to rotate the motor (in radians): "))

        qpos[dof_idx] = pos
        
    elif key == "m":

        if DEBUG:
            qpos_step = np.zeros(27)

            for i, joint in enumerate(finley.joints):

                print(qpos)
                print(qpos_step)

                qpos_step[i] = qpos[i]

                finley.control_dofs_position(qpos_step)

                # step a few frames so motion is visible
                for _ in range(20):
                    scene.step()
        
        else:

            print(qpos)
            
            finley.control_dofs_position(qpos)

            # step a few frames so motion is visible
            for _ in range(20):
                scene.step()

        qpos = finley.get_qpos().clone()
    
    else:

        print("Invalid Input, please try again.")