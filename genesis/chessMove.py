import genesis as gs
import numpy as np
import math
import time

gs.init()

scene = gs.Scene(
    show_viewer = True,
    viewer_options = gs.options.ViewerOptions(
        res           = (1280, 960),
        camera_pos    = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = 60,
    )
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

finley = scene.add_entity(
    gs.morphs.URDF(
        file = 'Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf',
        pos = (0.0, 0.0, .735),
        quat = (1, 0, 0, 0),
        fixed = True,
    ),
)

def get_user_target():
    print("\nEnter target coordinates (e.g., '0.3 0.0 0.5'):")
    try:
        user_input = input(">> ")
        coords = [float(x) for x in user_input.split()]
        if len(coords) != 3:
            print("Invalid input! Please enter 3 numbers.")
            return None
        return np.array(coords)
    except ValueError:
        print("Invalid numbers!")
        return None
    

scene.build()

joint_names = [
    "shoulderspin_left",
    "armlift_left",
    "elbowcurl_left",
    "handspin_left",
    "wristcurl_left",
    # "gripper_left"
]
left_arm_indices = [finley.get_joint(name).dofs_idx_local[0] for name in joint_names]

print(left_arm_indices)

standing_qpos = finley.get_qpos()

# get the end-effector link
end_effector = finley.get_link('hand_left')
print("end_effector position:", end_effector.get_pos())

# move to pre-grasp pose
while True:
        
    qpos = finley.inverse_kinematics(
        link = end_effector,
        pos  = get_user_target(), #xyz
    )
    
    if qpos is None:
        print("target out of reach")
        continue

    final_qpos = standing_qpos.clone()

    for i in left_arm_indices:
        final_qpos[i] = qpos[i]

    print('final_qpos:', final_qpos[left_arm_indices])

    path = finley.plan_path(
        qpos_goal = final_qpos,
        num_waypoints = 200,
    )

    if path is None:
        print("Path planning failed.")
        continue

    active_joints = [j for j in finley.joints if j.n_dofs > 0]
    print("------ ROBOT JOINTS ------")
    for i, joint in enumerate(active_joints):
        print(f"DOF Index {i}: {joint.name}")
    print("--------------------------")

    # execute the planned path
    for waypoint in path:
        finley.control_dofs_position(waypoint)
        scene.step()

    # allow robot to reach the last waypoint
    for i in range(100):
        scene.step()
    
    print("Reached Target Position!")
    print("end_effector position:", end_effector.get_pos())