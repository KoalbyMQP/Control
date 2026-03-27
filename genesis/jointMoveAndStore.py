# filepath: c:\Users\ccoto\Documents\GitHub\Control\genesis\jointMoveAndStore.py
import genesis as gs
import numpy as np
import torch
import json
from pathlib import Path

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
    ),
    rigid_options = gs.options.RigidOptions(
        enable_neutral_collision=True
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

scene.build()

# -------------------------
# Arm Configuration
# -------------------------
ARM_CONFIG = {
    "left": {
        "joints": ["shoulderspin_left", "armlift_left", "elbowcurl_left", "handspin_left", "wristcurl_left"],
        "ee_name": "gripper_left",
        "offset": 0.145,
    },
    "right": {
        "joints": ["shoulderspin_right", "armlift_right", "elbowcurl_right", "handspin_right", "wristcurl_right"],
        "ee_name": "gripper_right",
        "offset": -0.145,
    }
}

# -------------------------
# Storage Management
# -------------------------
STORAGE_FILE = "arm_positions.json"

def load_positions():
    if Path(STORAGE_FILE).exists():
        with open(STORAGE_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_positions(positions):
    with open(STORAGE_FILE, 'w') as f:
        json.dump(positions, f, indent=2)

def get_arm_dofs(arm_side):
    config = ARM_CONFIG[arm_side]
    dofs_idx = []
    for joint in finley.joints:
        if joint.name in config["joints"] and joint.n_dofs > 0:
            dofs_idx.extend(joint.dofs_idx_local)
    return dofs_idx

# -------------------------
# Main Control Loop
# -------------------------
positions = load_positions()

while True:
    print("\n=== Arm IK Controller ===")
    print("'l' - Move left arm")
    print("'r' - Move right arm")
    print("'s' - Show saved positions")
    print("'d' - Delete saved position")
    print("'q' - Quit")
    
    cmd = input("\nCommand: ").lower()
    
    if cmd == "q":
        save_positions(positions)
        break
    
    if cmd == "s":
        if positions:
            for label, data in positions.items():
                print(f"  {label}: {data['arm']}")
        else:
            print("No saved positions.")
        continue
    
    if cmd == "d":
        label = input("Position label to delete: ")
        if label in positions:
            del positions[label]
            save_positions(positions)
            print(f"Deleted '{label}'")
        continue
    
    if cmd in ["l", "r"]:
        arm_side = "left" if cmd == "l" else "right"
        config = ARM_CONFIG[arm_side]
        arm_dofs = get_arm_dofs(arm_side)
        ee_link = finley.get_link(config["ee_name"])
        
        if ee_link is None:
            print(f"Error: '{config['ee_name']}' not found!")
            continue
        
        x = float(input(f"Target X: "))
        y = float(input(f"Target Y: "))
        z = float(input(f"Target Z: "))
        
        target_pos = torch.tensor([x + config["offset"], y, z], dtype=torch.float32)
        
        # IK
        ik_result = finley.inverse_kinematics(
            link=ee_link,
            pos=target_pos,
            quat=np.array([0, 0, 0, 1]),
            dofs_idx_local=arm_dofs
        )
        
        # Path Planning
        path = finley.plan_path(
            qpos_goal=ik_result,
            num_waypoints=200,
        )
        
        print("Executing trajectory...")
        for waypoint in path:
            finley.control_dofs_position(waypoint)
            scene.step()
        
        for _ in range(100):
            scene.step()
        
        # Save option
        save_choice = input("Save this position? (y/n): ").lower()
        if save_choice == 'y':
            label = input("Position label: ")

            joint_qpos_dict = {}
            ik_list = ik_result.tolist()

            for joint in finley.joints:
                if joint.name in config["joints"] and joint.n_dofs > 0:
                    for idx in joint.dofs_idx_local:
                        joint_qpos_dict[joint.name] = float(ik_list[idx])

            positions[label] = {
                "arm": arm_side,
                "target": [x, y, z],
                "raw_qpos": ik_result.tolist(),
                "labeled_qpos": joint_qpos_dict

            }
            save_positions(positions)
            print(f"Saved as '{label}'")