import numpy as np
import genesis as gs
import time
import math
from koalbyArmController import RobotArmController
from scipy.spatial.transform import RigidTransform as trans
from scipy.spatial.transform import Rotation as R

class KoalbyThermometerController(RobotArmController):
    def __init__(self):
        super().__init__(urdf_file='SwappingURDF//urdf//SwappingURDF.urdf', ee_name='wrist_left')
        self.prev_target_point = None
        self.forehead_point = None

    def _setup_scene(self):
        """Initialize the Genesis scene and add entities."""
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
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
        
        # Add ground plane
        self.scene.add_entity(gs.morphs.Plane())
        
        # Add robot
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=self.urdf_file,
                pos=self.robot_pos,
                quat=(0, 0, 0, 1),
                fixed=True,
            )
        )

        #test_head = self.scene.add_entity(
        #    gs.morphs.Sphere(pos=(0.1, 0.3, 0.8), radius=0.02,fixed=True)
        #)
        
        shoulder_point = self.scene.add_entity(
            gs.morphs.Sphere(pos=(0.145, 0.0, 0.735), radius=0.02, fixed=True) 
        )

        target_point = self.scene.add_entity(
            gs.morphs.Sphere(pos=(0,0,0), radius=0.02, fixed=True)
        )

        

        self.scene.build()


    def _setup_arm_joints(self):
        """Configure arm joints and end effector."""
        #TODO: implement proper temp taking urdf, for end effector definitations, below it temporary
        # Get end effector link
        self.ee_link = self.robot.get_link(self.ee_name)
        print(f"End Effector Link: {self.ee_link.name if self.ee_link else 'Not found'}")
        
        #if self.ee_link is None:
        #    raise ValueError(f"End effector '{self.ee_name}' not found!")
        
        # Get joint indices for the selected arm
        # if self.ee_name == "gripper_left":
        #     self.offset = 0.145
        #     joint_names = self.left_arm_joints
        # elif self.ee_name == "gripper_right":
        #     self.offset = -0.145
        #     joint_names = self.right_arm_joints
        # else:
        #     raise ValueError("End effector must be 'gripper_left' or 'gripper_right'")

        if self.ee_name == "wrist_left":#temporary use of swapping URDF, should be gripper_left for final version
            self.offset = 0.145
            joint_names = self.left_arm_joints
        elif self.ee_name == "wrist_right":
            self.offset = -0.145
            joint_names = self.right_arm_joints
        else:
            raise ValueError("End effector must be 'gripper_left' or 'gripper_right'")

        
        # Extract DOF indices for arm joints
        self.arm_dofs_idx_local = []
        for joint in self.robot.joints:
            if joint.name in joint_names:
                if joint.n_dofs > 0:
                    self.arm_dofs_idx_local.extend(joint.dofs_idx_local)
        
        print(f"\nEnd Effector: {self.ee_name}")
        print(f"Arm DOFs local indices: {self.arm_dofs_idx_local}")
        
        # Get gripper joint DOF indices
        self.gripper_dofs_idx_local = []
        gripper_joint_name = self.ee_name  # "gripper_left" or "gripper_right"
        for joint in self.robot.joints:
            if joint.name == gripper_joint_name:
                if joint.n_dofs > 0:
                    self.gripper_dofs_idx_local.extend(joint.dofs_idx_local)
        
        print(f"Gripper DOFs local indices: {self.gripper_dofs_idx_local}")


    def update_therm_tracking(self, final_qpos,robot):
        robot.control_dofs_position(final_qpos)
        self.scene.step()
        #time.sleep(0.02)


    def point_therm(self, head_point: np.ndarray, robot):
        dist_from_head = 0.1 #hardcoded therm distance to forehead, temp value
        koalby_shoulder_pos = np.asarray((0.145, 0.0, 0.735))
        target_dist = np.linalg.norm(head_point - koalby_shoulder_pos)
        head_to_shoulder_unit_vector = -(head_point - koalby_shoulder_pos) / np.linalg.norm(head_point - koalby_shoulder_pos)
        near_next_point_thresh = 0.05#if current prev target pos is close,then skip planning and move directly to target
        print(f"Target point to shoulder distance: {np.linalg.norm(head_point - koalby_shoulder_pos)}")
        
        max_reach = 0.25 #HARDCODED VALUE, CHECK FOR VALIDITY
        if np.linalg.norm(head_point - koalby_shoulder_pos) > max_reach:
            dist_from_head = target_dist - max_reach #if target is beyond reach, set point at edge of reach
        target_point = head_point + head_to_shoulder_unit_vector * dist_from_head #target point is dist away from head towards shoulder
        
        target_quat = R.align_vectors([np.array([0,1, 0])], [-head_to_shoulder_unit_vector])[0].as_quat()

        if(self.prev_target_point is not None): #defaults to not near (ex. first time running)
            is_near_next_point = np.linalg.norm(self.prev_target_point - target_point) < near_next_point_thresh
        else:
            is_near_next_point = False

        joint_names = [
            "shoulderspin_right",
            "armlift_right",
            "elbowcurl_right",
            "handspin_right",
            "wristcurl_right",
        ]
        path = None

        left_arm_indices = [robot.get_joint(name).dofs_idx_local[0] for name in joint_names]
        standing_qpos = robot.get_qpos()
        end_effector = robot.get_link('wrist_right')#TODO change when correct URDF is built

        qpos = robot.inverse_kinematics(
            pos_tol = 0.00001,
            rot_tol = 0.00001,
            max_solver_iters = 100,
            link = end_effector,
            pos  = target_point,
            quat = target_quat,
            rot_mask = [False, True, False]# thermometer only needs to point, orientation about axis doesn matter
        )
        final_qpos = standing_qpos.clone()#start with standing pose
        for idx in left_arm_indices:#iteratevly replace
            final_qpos[idx] = qpos[idx]

        self.prev_target_point = target_point

        if not is_near_next_point:
            print("planning path")
            path = robot.plan_path(
                qpos_goal = final_qpos,
                num_waypoints = 30,
                resolution = 0.5,
                timeout = 5.0,
                ignore_collision = False,
                #smooth_path=False,
                max_nodes = 500
            )
            print("path planned")

            for wp in path:#within the individual waypoint, step through paths "waypoints"
                robot.control_dofs_position(wp)
                self.scene.step()
            #time.sleep(0.02) #small sleep to slow down motion for visibility
        else:
            print("calling direct move, skipping planning")
            self.update_therm_tracking(final_qpos, robot)
        # path = robot.plan_path(
        #         qpos_goal = final_qpos,
        #         num_waypoints = 30,
        #         resolution = 0.5,
        #         timeout = 5.0,
        #         ignore_collision = False,
        #         #smooth_path=False,
        #         max_nodes = 500
        # )

        # for wp in path:#within the individual waypoint, step through paths "waypoints"
        #     robot.control_dofs_position(wp)
        #     self.scene.step()
        #     #time.sleep(0.02) #small sleep to slow down motion for visibility

        self.scene.clear_debug_objects()#clear previous debug visuals
        self.scene.draw_debug_arrow(head_point, head_to_shoulder_unit_vector/10, radius=0.005, color=(1.0, 0.0, 0.0, 0.5))#unit vector made 1cm for visual clarity
        self.scene.draw_debug_sphere(pos=head_point, radius=0.01, color=(0.0, 1.0, 0.0, 0.5)) #head point in green
        self.scene.draw_debug_sphere(pos=target_point, radius=0.01)
        

        
            
        for _ in range(50):#after reaching final waypoint, step a few frames to allow robot to reach final position
                self.scene.step()
                #time.sleep(0.02)


    def point_at_moving_sim_head(self, robot):
        t = 0.0
        while True:
            head_point = np.array([0.1 + 0.1*math.sin(t), 0.52, 0.8 + 0.1*math.cos(t) ]) #moving head point
            self.forehead_point = head_point
            t += 0.1
            t = t % (2 * math.pi)#wraps t to circle
            print(head_point)
            self.scene.draw_debug_sphere(pos=head_point, radius=0.02, color=(0.0, 0.0, 1.0, 0.5)) #head point in green
            self.point_therm(self.forehead_point, robot)
            time.sleep(0.01)
    
    def update_forehead_point(self, point):#function called outside to update the internal forehead_point
        self.forehead_point

def init_genesis():
    print("Running Genesis test...")

    test_head_pos = [0.1, -0.1, 0.8]
    test_head_dir = [0.0, 1.0, 0.0]
    test_distance = 0.1

    # Initialize Genesis client
    gs.init(backend=gs.cuda)

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
    
    #print("here")
    # Create a simple scene with a head and a thermometer
    head = scene.add_entity(
        gs.morphs.Sphere(pos=test_head_pos, radius=0.05,fixed=True)
    )
    target_transform = find_target_point(test_head_pos, test_head_dir, test_distance)
    correction = R.from_euler('z', 90, degrees=True)
    target_quat = (target_transform.rotation * correction).as_quat(scalar_first=True)
    target_pos = target_transform.translation

    #rotate -90° about Y
    axis_correction = R.from_euler('y', -90, degrees=True)
    #correction applied AFTER pose rotation
    cylinder_rot = (target_transform.rotation * axis_correction).as_quat(scalar_first=True)
    thermometer = scene.add_entity(
        gs.morphs.Cylinder(pos=target_pos,quat=cylinder_rot, radius=0.01, height=0.1, fixed=True)
    )
    #thermometer_vector = scene.add_arrow(start=therm_position, end=therm_position + alg_result.rotation.apply([0.1, 0, 0]), color=[0, 1, 0])
    
    plane = scene.add_entity(
        gs.morphs.Plane()
    )
    #scene.build()
    
    finley = scene.add_entity(
        gs.morphs.URDF(
            file = 'SwappingURDF//urdf//SwappingURDF.urdf',
            pos = (0.0, 0.0, .735),
            quat = (1, 0, 0, 0),
            fixed = True,
        )
    )

    scene.build()

if __name__ == "__main__":
    ctrlr = KoalbyThermometerController()
    test_head_pos = [0.1, 0.3, 0.8]
    ctrlr.point_at_moving_sim_head(ctrlr.robot)
    #ctrlr.point_therm(test_head_pos, ctrlr.robot)
    # while(True):
    #     time.sleep(1)
    # #point_therm(test_head_pos)
