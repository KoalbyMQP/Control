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
        file = 'SwappingURDF//urdf//SwappingURDF.urdf',
        pos = (0.0, 0.0, .75),
        fixed = True,
    ),
)

scene.build()

#    [1=Shoulder, 5=ArmLift, 8=Elbow, 11=HandSpin, 15=WristCurl]
left_arm_indices = [1, 5, 8, 11, 15]
standing_qpos = finley.get_qpos()

# get the end-effector link
end_effector = finley.get_link('wrist_left')

# move to pre-grasp pose
qpos = finley.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.30, -0.70, 0.75]), #xyz
    quat = np.array([0, 1, 0, 0]),
)

final_qpos = standing_qpos.clone()

for i in left_arm_indices:
    final_qpos[i] = qpos[i]

path = finley.plan_path(
    qpos_goal     = final_qpos,
    num_waypoints = 200,
)

# active_joints = [j for j in finley.joints if j.n_dofs > 0]
# print("------ ROBOT JOINTS ------")
# for i, joint in enumerate(active_joints):
#     print(f"DOF Index {i}: {joint.name}")
# print("--------------------------")

# execute the planned path
for waypoint in path:
    finley.control_dofs_position(waypoint)
    scene.step()

# allow robot to reach the last waypoint
for i in range(100):
    scene.step()
