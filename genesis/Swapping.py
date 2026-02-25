import genesis as gs
import numpy as np
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
        file = 'SwappingURDF//urdf//SwappingURDF.urdf',
        pos = (0.0, 0.0, .735),
        quat = (1, 0, 0, 0),
        fixed = True,
    ),
)

SwappingStation = scene.add_entity(
    gs.morphs.URDF(
        file = 'SwappingStation//urdf//SwappingStationURDF.urdf',
        pos = (1.30, 0.91, 1.60),
        euler = (90, 90, 0), #x red, y green, z blue
        fixed = True,
    ),
)

scene.build()

joint_names = [
    "shoulderspin_left",
    "armlift_left",
    "elbowcurl_left",
    "handspin_left",
    "wristcurl_left",
]
left_arm_indices = [finley.get_joint(name).dofs_idx_local[0] for name in joint_names]

standing_qpos = finley.get_qpos()

end_effector = finley.get_link('wrist_left')

waypoints = [
    np.array([0.08, -0.2, 0.65]),   # Point 1
    np.array([0.08, -0.25, 0.6]),   # Point 2
    np.array([0.1, -0.15, 0.65]),  # Point 3
    np.array([0.08, 0.05, 0.5]),   # Point 4 (Return closer to start)
]


# Fixed orientation of end effector
target_orientation = np.array([0.9239, 0.0, -0.3827, 0.0]) 

for i, target_pos in enumerate(waypoints):
    print(f"\n--- Moving to Waypoint {i+1}: {target_pos} ---")

    qpos = finley.inverse_kinematics(
        link = end_effector,
        pos  = target_pos,
        quat = target_orientation,
    )
    
    if qpos is None:
        print(f"Waypoint {i+1} is out of reach")
        continue

    final_qpos = standing_qpos.clone()
    for idx in left_arm_indices:
        final_qpos[idx] = qpos[idx]


    path = finley.plan_path(
        qpos_goal = final_qpos,
        num_waypoints = 50, 
    )

    if path is None:
        print(f"Path planning failed for Waypoint {i+1}.")
        continue

    for waypoint in path:
        finley.control_dofs_position(waypoint)
        scene.step()

    for _ in range(100):
        scene.step()
    
    print(f"Reached Waypoint {i+1}.")
    
    time.sleep(0.5) 

print("\nAll waypoints completed.")

while True:
    scene.step()