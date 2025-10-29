import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import sys, time, math, array
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
# from backend.Testing import finlyViaPoints as via



# Creating URDF chain for left arm 
left_arm_chain = Chain.from_urdf_file(
    "backend/KoalbyHumanoid/Simulation Files/Humanoid_URDF_9-10/urdf/Humanoid_URDF_9-10.urdf",
    base_elements=['shoulder_left', 'shoulderspin_left'],
    active_links_mask=[False, True, True, True, True, True, True]  
)

# creating URDF chain for camera
camera = Chain.from_urdf_file(
    "backend/KoalbyHumanoid/Simulation Files/Humanoid_URDF_9-10/urdf/Humanoid_URDF_9-10.urdf",
    base_elements=['neck', 'neckturn']   
)

#forward kinematics for camera chain
camera_angles=np.array([0,0,0])
camera_frame_transformation=camera.forward_kinematics(camera_angles)


# Edit to declare if you are testing the sim or the real robot
is_real = False
robot = Robot2(is_real)
print("Setup Complete")


# positions


final_position=np.array([0,  0, 0])



#Starting Angles

# robot.motors[25].target = (math.radians(30), 'P')
# robot.motors[26].target = (math.radians(10), 'P')


robot.motors[5].target = (math.radians(0), 'P')
robot.motors[6].target = (math.radians(0), 'P')
robot.motors[7].target = (math.radians(0), 'P')
robot.motors[8].target = (math.radians(0), 'P')
robot.motors[9].target = (math.radians(0), 'P')
robot.motors[10].target = (math.radians(0), 'P')



ik_solution_2=np.array([0,0,0,0,0,0,0])


# centering all angles to zero
prevTime = time.time()
simStartTime = time.time()
while time.time() - simStartTime < 2:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()


# conversion of final points from camera coordinate systm to rorbot coordinate system 
final_points=np.array([0, 0, 0])
B=np.array([[final_points[0]],[final_points[1]],[final_points[2]],[1]])
A= camera_frame_transformation
final_points=np.array([0,0, 0])
C = np.dot(A, B)

print(C)


end_effector_start_pos = [.49076,  -.08197, .76541]
# end_effector_start_pos = [0.1151, -0.08197, 0.39374]
end_position = [0.15, -0.35, 0.5]
end_effector_z_offset = 0.1
end_effector_y_offset = 0.0
end_effector_x_offset = 0.007

leftArmTraj = [
    [[0,0,0], [20, 20, 20]],
    [[end_effector_start_pos[0],  end_effector_start_pos[1], end_effector_start_pos[2]],
   [end_position[0]+end_effector_x_offset, end_position[1]+end_effector_y_offset, end_position[2]+end_effector_z_offset]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]

final_position=left_arm_chain.forward_kinematics(ik_solution_2)

lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])
import numpy as np

def within_threshold(p1, p2, threshold):
    p1 = np.asarray(p1).reshape(-1)
    p2 = np.asarray(p2).reshape(-1)
    dist = np.linalg.norm(p1 - p2)
    return dist <= threshold, dist

threshold = 3e-4
alpha = 0.5

startTime = time.time()
target_orientation = np.array([0.0, -1.0, 1.0])

while time.time() - startTime < 20:
        
     
        target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter=0.01)
        fk_solution = left_arm_chain.forward_kinematics(ik_solution)
        fk_taskspace_coords = [fk_solution[0][3], fk_solution[1][3], fk_solution[2][3]]
        # print(within_threshold(target_position_2, fk_taskspace_coords, threshold))
        ok, err = within_threshold(target_position_2, fk_taskspace_coords, threshold)
        if ok:
            print("I am keeping the IK moving")
            ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter=0.01)
            ik_solution_2=ik_solution
            motor_angle_task=ik_solution
            # motor_angle_task_prev = ik_solution
            # motor_angle_task = alpha * ik_solution + (1 - alpha) * motor_angle_task_prev
            # motor_angle_task_prev = motor_angle_task

        else:
            print("I am reinitializing the IK")
            ik_solution_updated = left_arm_chain.inverse_kinematics(fk_taskspace_coords, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter=0.01)
            ik_solution_2 = ik_solution_updated
            motor_angle_task = ik_solution_updated
            # motor_angle_task_prev = ik_solution_updated
            # motor_angle_task = alpha * ik_solution_updated + (1 - alpha) * motor_angle_task_prev
            # motor_angle_task_prev = motor_angle_task

        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        
        # print(motor_angle_task)

        # robot.IMUBalance(0, 0)
        robot.moveAllToTarget()

# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# Step 2 -> Moving Down to Piece

if time.time() - startTime > 20:
    pick_up_motion = [end_position[0]+ end_effector_x_offset, end_position[1]+end_effector_y_offset, end_position[2]+(end_effector_z_offset/2)]
    leftArmTraj = [
    [[0,0,0], [5, 5, 5]],
    [[end_position[0]+end_effector_x_offset,  end_position[1]+end_effector_y_offset, end_position[2]+end_effector_z_offset],
   [pick_up_motion[0], pick_up_motion[1] , pick_up_motion[2]]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]
final_position=left_arm_chain.forward_kinematics(ik_solution_2)

lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])

startTime = time.time()
while time.time() - startTime < 5:
        target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter=0.01)
        ik_solution_2=ik_solution
        motor_angle_task=ik_solution

        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        
        # print(motor_angle_task)

        # robot.IMUBalance(0, 0)
        robot.moveAllToTarget()

# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# Step 3 -> Moving up to intermediate position

if time.time() - startTime > 5:
    pick_up_motion_2 = [end_position[0]+ end_effector_x_offset, end_position[1]+end_effector_y_offset, end_position[2]+end_effector_z_offset]
    leftArmTraj = [
    [[0,0,0], [5, 5, 5]],
    [[pick_up_motion[0], pick_up_motion[1] , pick_up_motion[2]],
   [pick_up_motion_2[0], pick_up_motion_2[1] , pick_up_motion_2[2]]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]
final_position=left_arm_chain.forward_kinematics(ik_solution_2)

lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])

startTime = time.time()
while time.time() - startTime < 5:
        target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter=0.01)
        ik_solution_2=ik_solution
        motor_angle_task=ik_solution

        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        
        # print(motor_angle_task)

        # robot.IMUBalance(0, 0)
        robot.moveAllToTarget()

# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# Step 4 -> Moving to opponent side

if time.time() - startTime > 5:
    move_forward_motion = [pick_up_motion_2[0], -0.428, 0.615+end_effector_z_offset]
    leftArmTraj = [
    [[0,0,0], [20, 20, 20]],
    [[pick_up_motion_2[0], pick_up_motion_2[1] , pick_up_motion_2[2]],
   [move_forward_motion[0], move_forward_motion[1] , move_forward_motion[2]]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]
final_position=left_arm_chain.forward_kinematics(ik_solution_2)

lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])

startTime = time.time()
while time.time() - startTime < 20:
        target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter = 0.01)
        ik_solution_2=ik_solution
        motor_angle_task=ik_solution

        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        
        # print(motor_angle_task)

        # robot.IMUBalance(0, 0)
        robot.moveAllToTarget()

# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# Step 5 -> Moving down to pick up the piece

if time.time() - startTime > 5:
    leftArmTraj = [
    [[0,0,0], [20, 20, 20]],
    [[move_forward_motion[0], move_forward_motion[1] , move_forward_motion[2]],
   [move_forward_motion[0], move_forward_motion[1]+end_effector_y_offset, pick_up_motion[2]]],
    [[0,0,0], [0,0,0]],
    [[0,0,0], [0,0,0]]
]
final_position=left_arm_chain.forward_kinematics(ik_solution_2)

lArm_tj_joint = TrajPlannerTime(leftArmTraj[0], leftArmTraj[1], leftArmTraj[2], leftArmTraj[3])

startTime = time.time()
while time.time() - startTime < 20:
        target_position_task = lArm_tj_joint.getQuinticPositions(time.time() - startTime)
        target_position_2 = np.array([(target_position_task[0]), (target_position_task[1]), (target_position_task[2])])
        ik_solution = left_arm_chain.inverse_kinematics(target_position_2, initial_position=ik_solution_2, target_orientation=target_orientation, orientation_mode="Y", regularization_parameter = 0.01)
        ik_solution_2=ik_solution
        motor_angle_task=ik_solution

        robot.motors[5].target = (motor_angle_task[1], 'P')
        robot.motors[6].target = (motor_angle_task[2], 'P')
        robot.motors[7].target = (motor_angle_task[3], 'P')
        robot.motors[8].target = (motor_angle_task[4], 'P')
        robot.motors[9].target = (motor_angle_task[5], 'P')
        
        print(motor_angle_task)

        # robot.IMUBalance(0, 0)
        robot.moveAllToTarget()


# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------
# Code to plot out workspace
 

# # Put this near the top with your imports
# import numpy as np
# from mpl_toolkits.mplot3d import Axes3D
# from ikpy.chain import Chain
# from tqdm import tqdm 
# import matplotlib.pyplot as plt

# def sample_joint_space(n_samples, joint_limits):
#     """
#     n_samples: int
#     joint_limits: list of (min, max) for each joint (excluding the fixed root element)
#     returns: array (n_samples, n_joints)
#     """
#     n_joints = len(joint_limits)
#     samples = np.random.rand(n_samples, n_joints)
#     for j in range(n_joints):
#         lo, hi = joint_limits[j]
#         samples[:, j] = lo + samples[:, j] * (hi - lo)
#     return samples

# def fk_positions_for_samples(chain: Chain, samples):
#     """
#     chain: ikpy.Chain for the arm (with same order of moving joints as samples)
#     samples: (N, n_joints) numpy
#     returns: (N,3) end-effector positions in chain base frame
#     """
#     points = []
#     for q in samples:
#         # ikpy expects a full vector with first element = 0 (dummy) if URDF includes root
#         # Many Chains include a fixed element; adapt as needed. Here we try directly.
#         try:
#             T = chain.forward_kinematics(q)
#         except Exception:
#             # If chain expects full vector with extra leading 0:
#             padded = np.concatenate(([0.0], q))
#             T = chain.forward_kinematics(padded)
#         pos = T[:3, 3]
#         points.append(pos)
#     return np.array(points)

# # --- cheap sphere-approx self collision ---
# def build_body_spheres(chain: Chain, radii_by_link=None):
#     """
#     Build sphere approximations centered at link frames (link origins).
#     radii_by_link: dict mapping link_name -> radius (meters). If None, use a small default or
#                    infer from URDF visuals manually.
#     returns list of (link_frame_index, center_func, radius) where center_func(q) gives center.
#     """
#     centers_idx = []
#     # We will use forward_kinematics of each link frame.
#     link_names = [j.name for j in chain.links]
#     # Provide default radii if not provided:
#     if radii_by_link is None:
#         radii_by_link = {name: 0.07 for name in link_names}  # tweak per robot
#     # prepare functions to compute center position for each link index
#     for idx, link in enumerate(chain.links):
#         def center_fn(q, idx_local=idx):
#             # compute FK up to link idx_local
#             # ikpy doesn't expose per-link FK easily, but chain.links[...] has frames
#             # We'll do full FK and extract transformation of the link's frame if available.
#             T = chain.forward_kinematics(q, full_kinematics=True)
#             # full_kinematics returns list of transforms for each frame
#             # try to use that; else fallback to whole transform
#             if isinstance(T, (list, tuple)) and len(T) > idx_local:
#                 tlink = T[idx_local]
#                 return tlink[:3, 3]
#             else:
#                 return chain.forward_kinematics(q)[:3, 3]
#         centers_idx.append((link.name, center_fn, radii_by_link.get(link.name, 0.07)))
#     return centers_idx

# def is_self_collision(q, sphere_list, min_clearance=0.02):
#     """
#     sphere_list: list of (name, center_fn, radius)
#     returns True if any pair of spheres overlap (collision) or are too close
#     """
#     centers = [fn(q) for (_n, fn, r) in sphere_list]
#     radii = [r for (_n, _fn, r) in sphere_list]
#     centers = np.array(centers)
#     for i in range(len(centers)):
#         for j in range(i + 1, len(centers)):
#             d = np.linalg.norm(centers[i] - centers[j])
#             if d < (radii[i] + radii[j] + min_clearance):
#                 return True
#     return False

# # --- voxelize and plot ---
# def voxelize_points(points, voxel_size=0.02, grid_margin=0.2):
#     mins = points.min(axis=0) - grid_margin
#     maxs = points.max(axis=0) + grid_margin
#     dims = np.ceil((maxs - mins) / voxel_size).astype(int)
#     # convert points to voxel indices
#     idxs = np.floor((points - mins) / voxel_size).astype(int)
#     occupancy = {}
#     for ix, iy, iz in idxs:
#         occupancy[(ix, iy, iz)] = occupancy.get((ix, iy, iz), 0) + 1
#     voxels = np.array(list(occupancy.keys()))
#     counts = np.array(list(occupancy.values()))
#     # convert back to centers
#     centers = mins + (voxels + 0.5) * voxel_size
#     return centers, counts

# def plot_point_cloud(points, s=2):
#     fig = plt.figure(figsize=(8,8))
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(points[:,0], points[:,1], points[:,2], s=s)
#     ax.set_xlabel('Z'); ax.set_ylabel('Y'); ax.set_zlabel('X')
#     plt.show()

# left_arm_chain = Chain.from_urdf_file(
#     "backend/Testing/FinleyJNEWARMS_2024_straight_4.urdf",
#     base_elements=['shoulder1_left', 'shoulder1_left'],
#     active_links_mask=[False, True, True, True, True, True, True]  
# )


# # Determine joint limits: you should get these from your URDF or robot config.
# # For demo, assume 7 dof from joint 1..6 (adjust count to your Chain).
# # Format: [(min, max), ...] in radians
# joint_limits = [(-2.8, 2.8), (-1.5, 1.5), (-2.8, 2.8), (-2.0, 2.0), (-2.5, 2.5), (-3.0, 3.0)]

# N = 8000
# samples = sample_joint_space(N, joint_limits)

# # Optionally filter by self-collision using the sphere approximation:
# valid_points = []
# for q in tqdm(samples):
#     # just collect positions
#     pos = left_arm_chain.forward_kinematics(np.concatenate(([0.0], q)))[:3,3]
#     valid_points.append(pos)
# valid_points = np.array(valid_points)
# print("Valid reachable points:", valid_points.shape[0])


# # Make voxel occupancy for visualization
# centers, counts = voxelize_points(valid_points, voxel_size=0.02)
# plot_point_cloud(centers)