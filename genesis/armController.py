import numpy as np
import heapq
import genesis as gs

class armController():
    def __init__(self, humanoid, scene):
        self.humanoid = humanoid
        self.scene = scene

        self.left_arm_joints = [
            "shoulderspin_left",
            "armlift_left",
            "elbowcurl_left",
            "handspin_left",
            "wristcurl_left"
        ]

        self.right_arm_joints = [
            "shoulderspin_right",
            "armlift_right",
            "elbowcurl_right",
            "handspin_right",
            "wristcurl_right"
        ]

    # ------------------- Fast Workspace A* -------------------
    def astar_with_collision(self, EE, joint_indices, start_xyz, goal_xyz, voxel=0.05, max_iters=5000):
        """Fast collision-aware A* in workspace (XYZ)"""
        def to_grid(pos):
            pos_np = pos.cpu().numpy() if hasattr(pos, "cpu") else np.array(pos)
            return tuple((pos_np / voxel).astype(int))

        def to_world(idx):
            return np.array(idx) * voxel

        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        start = to_grid(start_xyz)
        goal = to_grid(goal_xyz)

        neighbors = [(dx, dy, dz)
                     for dx in [-1, 0, 1]
                     for dy in [-1, 0, 1]
                     for dz in [-1, 0, 1]
                     if not (dx == 0 and dy == 0 and dz == 0)]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        current_qpos = self.humanoid.get_qpos()
        current_qpos = current_qpos.cpu().numpy() if hasattr(current_qpos, "cpu") else np.array(current_qpos)

        iters = 0

        while open_set and iters < max_iters:
            _, current = heapq.heappop(open_set)

            # Early exit if close enough
            if heuristic(to_world(current), goal_xyz) < voxel:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return [to_world(p) for p in path[::-1]]

            for dx, dy, dz in neighbors:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                world_point = to_world(neighbor)

                # Check IK feasibility and collisions (fast)
                ik_solution = self.humanoid.inverse_kinematics(
                    link=EE,
                    pos=world_point,
                    dofs_idx_local=joint_indices,
                    init_qpos=current_qpos
                )
                if ik_solution is None:
                    continue
                ik_solution = ik_solution.cpu().numpy() if hasattr(ik_solution, "cpu") else np.array(ik_solution)

                self.humanoid.set_qpos(ik_solution)
                collisions = self.humanoid.detect_collision()
                if collisions is not None and len(collisions) > 0:
                    continue

                tentative_g = g_score[current] + heuristic(current, neighbor)
                neighbor_key = neighbor
                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    came_from[neighbor_key] = current
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor_key))

            iters += 1

        print("No collision-free workspace path found")
        return None

    # ------------------- Smooth Path Interpolation -------------------
    def interpolate_path(self, path, step=0.02):
        smooth_path = []
        for i in range(len(path)-1):
            start = path[i]
            end = path[i+1]
            dist = np.linalg.norm(end - start)
            n_steps = max(int(dist/step), 1)
            for t in np.linspace(0, 1, n_steps, endpoint=False):
                smooth_path.append(start*(1-t) + end*t)
        smooth_path.append(path[-1])
        return smooth_path

    # ------------------- Move Arm -------------------
    def move_to_XYZ(self, ee_link, goal_xyz):
        joint_indices = []

        if ee_link == "left":
            EE = self.humanoid.get_link("gripper_left")
            joint_names = self.left_arm_joints
            quat = np.array([0, 0, 1, 1])
        elif ee_link == "right":
            EE = self.humanoid.get_link("gripper_right")
            joint_names = self.right_arm_joints,
            quat = np.array([0, 0, -1, 1])
        else:
            print("Invalid EE link")
            return

        # Find joint indices
        for name in joint_names:
            found = False
            for i, joint in enumerate(self.humanoid.joints):
                if joint.name == name:
                    joint_indices.append(i)
                    found = True
                    break
            if not found:
                print(f"Joint not found: {name}")

        current_qpos = self.humanoid.get_qpos()
        current_qpos = current_qpos.cpu().numpy() if hasattr(current_qpos, "cpu") else np.array(current_qpos)
        current_xyz = EE.get_pos()
        current_xyz = current_xyz.cpu().numpy() if hasattr(current_xyz, "cpu") else np.array(current_xyz)

        print("Planning Path")

        # Plan coarse workspace path
        coarse_path = self.astar_with_collision(EE, joint_indices, current_xyz, goal_xyz)
        if coarse_path is None:
            return

        print("Interpolating Path")

        # Smooth / interpolate path
        path = self.interpolate_path(np.array(coarse_path))

        print(f"Executing {len(path)} waypoints...")

        # Execute the path
        for point in path:
            ik_solution = self.humanoid.inverse_kinematics(
                link=EE,
                pos=point,
                quat=quat,
                dofs_idx_local=joint_indices,
                init_qpos=current_qpos
            )
            if ik_solution is None:
                continue
            ik_solution = ik_solution.cpu().numpy() if hasattr(ik_solution, "cpu") else np.array(ik_solution)
            current_qpos = ik_solution.copy()
            self.humanoid.control_dofs_position(ik_solution)
            self.scene.step()

        # Extra frames to stabilize
        for _ in range(30):
            self.scene.step()

    # ------------------- User Target -------------------
    def get_user_target(self):
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

# Example Usage

gs.init()

scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        res=(1280, 960),
        camera_pos=(3.5, 2.0, 2.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=40,
    )
)

scene.add_entity(gs.morphs.Plane())

finley = scene.add_entity(
    gs.morphs.URDF(
        file="Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf",
        pos=(0.0, 0.0, 0.735),
        quat=(0, 0, 0, 1),
        fixed=True,
    )
)

cart = scene.add_entity(
    gs.morphs.Box(
        size=(0.6, 0.6, 0.6),
        pos=(0.0, 0.5, 0.3),
        fixed=True
    )
)

scene.build()

controller = armController(finley, scene)

goal_xyz = np.array([0.0, 0.4, 0.75])

controller.move_to_XYZ("left", goal_xyz)