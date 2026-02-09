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
        file = 'genesis//Humanoid_URDF_9-10//urdf//Humanoid_URDF_9-10.urdf',
        pos = (0.0, 0.0, .75),
        fixed = True
    ),
)

scene.build()

# get the end-effector link
end_effector = finley.get_link('gripper_left')

# move to pre-grasp pose
qpos = finley.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.70, 0.70, 0.50]),
    quat = np.array([1, 0, 0, 0]),
)

# gripper open pos
qpos[-2:] = 0.04
path = finley.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)

# execute the planned path
for waypoint in path:
    finley.control_dofs_position(waypoint)
    scene.step()

# allow robot to reach the last waypoint
for i in range(100):
    scene.step()
