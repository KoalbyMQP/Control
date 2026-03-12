import numpy as np
import genesis as gs
import time
import math
from scipy.spatial.transform import RigidTransform as trans
from scipy.spatial.transform import Rotation as R

def jacobian_numeric(current,step,target, shoulder_pos, max_arm_length):
    J = np.zeros((6, 6))
    for i in range(6):
        dv = np.zeros(6)
        dv[i] = step
        r_plus = residual_(apply_update(current, dv),target, shoulder_pos, max_arm_length)
        r_minus = residual_(apply_update(current, -dv),target, shoulder_pos, max_arm_length)
        J[:, i] = (r_plus - r_minus) / (2 * step)
    return J

def residual(current,target):
    
    terr = current.inv() * target
    trans_err = terr.translation
    rot_err = terr.rotation.as_rotvec()
    #print("Translation Error:", trans_err)
    return np.hstack((trans_err, rot_err))

def residual_(current, target,
             shoulder_pos=None, max_arm_length=0.5,
             w_trans=1.0, w_rot=1.0,
             w_shoulder=1.0):
    """
    Returns a 6D residual: [dx, dy, dz, rx, ry, rz]
    but translation part includes a shoulder-distance penalty as a *vector push*.
    """

    terr = current.inv() * target
    trans_err = terr.translation              # meters
    rot_err   = terr.rotation.as_rotvec()     # radians

    # Base weighted pose residual (same shape as before)
    r_trans = w_trans * trans_err
    r_rot   = w_rot   * rot_err

    if shoulder_pos is None:
        return np.hstack((r_trans, r_rot))

    # Distance from shoulder
    p = current.translation
    s = np.asarray(shoulder_pos, dtype=float)
    v = p - s
    d = np.linalg.norm(v)

    # Unit vector pointing away from shoulder
    u = v / (d + 1e-12)

    L = float(max_arm_length)
    frac = d / (L + 1e-12)

    # 0 until you get close, then linearly ramps up
    over = max(0.0, frac - 1)
    push_mag = w_shoulder * over * L
    
    r_trans = r_trans + push_mag * u

    return np.hstack((r_trans, r_rot))

def apply_update(tf, delta6):
    #delta6 is [rx, ry, rz, vx, vy, vz] exponential coords
    delta_tf = trans.from_exp_coords(delta6)
    return delta_tf * tf

def find_target_point(#TUNABLE CONSTANTS HERE =========================================================================================================================
    head_pos,
    head_dir,#vector
    distance_d,#ideal distance from thermometer to head
    shoulder_pos = np.array([0.0, 0.0, 0.0]),#shoulder position, used in error that is minimized
    max_arm_length=1.0,#length of arm, generally is max distance to check if Koalby can reach so defaulted to 0.5m
    init_position=np.array([0.0, 0.0, 0.0]),#position algorithm starts from
    blend_n=0.85, #blending factor for final error metric, 1.0 = only target error, 0.0 = only shoulder distance error
    lam=5e-6,#damping factor for least-squares
    eps=1e-6,#finite difference step for numerical Jacobian
    tol=1e-4,#convergence tolerance on residual norm
    max_iter=50,#max iterations of algorithm
):
    
    def unit(v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-12 else np.zeros_like(v)

    H = np.asarray(head_pos, dtype=float)
    d = np.asarray(head_dir, dtype=float)
    d = unit(d)
    if np.linalg.norm(d) < 1e-12:
        raise ValueError("head_dir must be a non-zero vector")

    # places target point distance distance_d along head_dir from head_pos
    Vt_point = H + float(distance_d) * d

    #quick check if its within reach
    # if np.linalg.norm(Vt_point - shoulder_pos) > max_arm_length:
    #     print("Target point is out of reach")
    #     exit()

    # initialize current transform
    curr_tf = trans.from_translation(np.asarray(init_position))

    #build target rotation where target vector is along x-axis
    x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
    tgt_dir = unit(H - Vt_point)#direction from target point to head
    dot = np.clip(np.dot(x_axis, tgt_dir), -1.0, 1.0)
    if np.allclose(tgt_dir, x_axis):#if target direction is already along x-axis
        rot = trans.from_translation(np.zeros(3)).rotation
    elif np.allclose(tgt_dir, -x_axis):#if target direction is opposite x-axis
        rot = R.from_rotvec(np.pi * np.array([0.0, 1.0, 0.0]))
    else:
        axis = np.cross(x_axis, tgt_dir)
        axis = axis / (np.linalg.norm(axis) + 1e-16)
        angle = math.acos(dot)

        rot = R.from_rotvec(axis * angle)

    target_tf = trans.from_components(rotation=rot, translation=Vt_point)#assembled target transform

    for k in range(max_iter):
        print(f"Iteration {k+1}")
        print("Current Transform:", curr_tf)
        r = residual_(curr_tf,target_tf, shoulder_pos, max_arm_length)
        #print("Residual:", r)
        err_norm = np.linalg.norm(r)

        if shoulder_pos is not None:
            shoulder_dist = np.linalg.norm(curr_tf.translation - shoulder_pos)
            dist_err = -1/(shoulder_dist/max_arm_length - 1 + 1e-12) #error term that grows as shoulder distance approaches max arm length

            combined_err = blend_n * err_norm + (1.0 - blend_n) * dist_err
        else:
            combined_err = err_norm
        print(f"Combined Error: {combined_err:.6f} (Residual Norm: {err_norm:.6f}, Shoulder Dist: {shoulder_dist:.6f})")
        if combined_err < tol:
            break

        J = jacobian_numeric(curr_tf, eps, target_tf, shoulder_pos, max_arm_length)
        JJt = J @ J.T
        reg = (lam ** 2) * np.eye(JJt.shape[0])
        try:#attempts inverse, settles for pseudo-inverse if singular
            inv = np.linalg.inv(JJt + reg)
        except np.linalg.LinAlgError:
            inv = np.linalg.pinv(JJt + reg)
        Jstar = J.T @ inv
        delta = -Jstar @ r
        curr_tf = apply_update(curr_tf, delta)
        #time.sleep(0.1)

        

    return curr_tf #returns tf object, should proably be unpacked

def genesis_test():
    print("Running Genesis test...")

    test_head_pos = [0.3, 0.0, 1.2]
    test_head_dir = [-1.0, 0.0, 0.0]
    test_distance = 0.1

    # Initialize Genesis client
    gs.init()

    scene = gs.Scene(
        show_viewer = True,
        viewer_options = gs.options.ViewerOptions(
            res           = (1280, 960),
            camera_pos    = (0, 2, 1.5),
            camera_lookat = (0.0, 0.0, 0.5),
            camera_fov    = 40,
            max_FPS       = 60,
        )
    )
    
    print("here")
    # Create a simple scene with a head and a thermometer
    head = scene.add_entity(
        gs.morphs.Sphere(pos=test_head_pos, radius=0.05,fixed=True)
    )
    alg_result = find_target_point(test_head_pos, test_head_dir, test_distance)

    print("Resulting Transform:")
    print("Translation:", alg_result.translation)
    print("Rotation (as rotvec):", alg_result.rotation.as_rotvec())

    therm_position = alg_result.translation
    #therm_orientation_as_vector = alg_result.rotation.as_rotvec()

    #rotate -90° about Y
    axis_correction = R.from_euler('y', -90, degrees=True)

    #correction applied AFTER pose rotation
    final_rot = alg_result.rotation * axis_correction
    thermometer = scene.add_entity(
        gs.morphs.Cylinder(pos=therm_position,quat=final_rot.as_quat(scalar_first=True), radius=0.01, height=0.1, fixed=True)
    )
    #thermometer_vector = scene.add_arrow(start=therm_position, end=therm_position + alg_result.rotation.apply([0.1, 0, 0]), color=[0, 1, 0])
    
    plane = scene.add_entity(
        gs.morphs.Plane()
    )
    scene.build()

    while(True):
        time.sleep(0.1)
        scene.step()

if __name__ == "__main__":
    

    genesis_test()
