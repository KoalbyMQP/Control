import genesis as gs
import numpy as np

class koalbyMove():

    def __init__(self, urdf_path):
        self = self
        self.urdf_path = urdf_path

    def loadScene(self):

        gs.init()

        scene = gs.Scene(

            # Set viewer options
            show_viewer = True,
            viewer_options = gs.options.ViewerOptions(
                res           = (1280, 960),
                camera_pos    = (3.5, 0.0, 2.5),
                camera_lookat = (0.0, 0.0, 0.5),
                camera_fov    = 40,
                max_FPS       = 60,
            )
        )

        # Add the ground plane
        plane = scene.add_entity(
            gs.morphs.Plane(),
        )

        # Add robot Humanoid Robot
        robot = scene.add_entity(
            gs.morphs.URDF(
                file = self.urdf_path,
                pos = (0.0, 0.0, .75),
                fixed = True
            ),
        )

        scene.build()

        self.scene = scene
        self.robot = robot
    
    def makeMove(self, scene, robot, goal, end_effector):

        # Determine end effector link
        if (end_effector == 'right'):
            ee = robot.get_link('gripper_right')

        elif(end_effector == 'left'):
            ee = robot.get_link('gripper_left')

        else:
            print("Invalid End Effector. Enter a valid end effector (right or left)")
            return -1
        
        # handle inverse kinematics
        qpos = robot.inverse_kinematics(
            link = ee,
            pos  = np.array(goal),
            quat = np.array([1, 0, 0, 0]),
        )

        # plan a motion path, motion duration = num_waypoints / 100
        path = robot.plan_path(
            qpos_goal     = qpos,
            num_waypoints = 200, # 2s duration
        )

        # execute the planned path
        for waypoint in path:
            robot.control_dofs_position(waypoint)
            scene.step()

        # allow robot to reach the last waypoint
        for i in range(100):
            scene.step()

    def motionRoutine(self):

        self.loadScene()

        # variable for continuing the sim
        runStatus = True

        while(runStatus):

            # Wait for a move input
            posString = input("Please enter a goal position (with the format 'x y z') or 'q' to quit")

            if (posString == 'q'):
                runStatus = False

            else:
                # Convert input into an array and convert strings to ints
                pos = np.array(posString.split(), dtype = float)

                end_effector = input("Which hand would you like to use? (right or left)")

                self.makeMove(self.scene, self.robot, pos, end_effector)
        
        # Let the sim run for another second, then close
        for i in range(100):
            self.scene.step()