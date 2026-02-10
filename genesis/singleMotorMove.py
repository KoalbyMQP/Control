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
        quat = (0, 0, 0, 1),
        fixed = True,
    ),
)

def getMove():

    print("------ ROBOT JOINTS ------")
    for i, joint in finley.joints:
        print(f"DOF Index {i}: {joint.name}")
    print("--------------------------")

    print("\nEnter the index of the joint you would like to move:\n")
    joint_idx = input(">> ")

    print("\nEnter what position (in radians) you would like to set the joint to:\n")
    joint_pos = input(">> ")

    return joint_idx, joint_pos

while True:

    joint_idx, joint_pos = getMove()

    q_desired = np.zeros(len(finley.joints))

    for i in q_desired:
        if i == joint_idx:
            q_desired[i] = joint_pos

    finley.control_dofs_position(q_desired)
