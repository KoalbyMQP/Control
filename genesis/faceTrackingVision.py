import threading
import time
import numpy as np
import genesis as gs
import math
from _face_mesh_GPT import faceTracker
from takeTemp import KoalbyThermometerController

tracker = faceTracker()
controller = KoalbyThermometerController()

tracking_thread = threading.Thread(target=tracker.run, daemon=True)  # start face tracking on another background thread
tracking_thread.start()

try:
    time.sleep(1)  # give the tracker a moment to start up and get the first forehead position
    while True:
        try:
            pos = np.asarray(tracker.forehead_pos) / 1000.0  # convert from cm to m for Genesis
        except TypeError:
            continue
        #LINE BELOW CONTAINS UNVERIFIED ROBOT CAMERA HEIGHT, AND DOES NOT CONSIDER ANY ROTATION OF THE HEAD, JUST TAKES THE FOREHEAD POINT AND TRANSFORMS TO GENESIS COORDINATES
        pos = [pos[0], pos[2], -pos[1] + 0.8]  # re-order to match Genesis coordinate system
        print("forehead point:", pos)
        controller.scene.draw_debug_sphere(pos, radius=0.02, color=(1, 0, 0))  # draw a red sphere at the forehead point for debugging
        controller.point_therm(pos, controller.robot)
        time.sleep(0.1)  # small sleep to prevent spamming the scene with too many debug spheres
        controller.scene.clear_debug_objects()
except KeyboardInterrupt:
    print("KeyboardInterrupt received, stopping camera...")
finally:
    tracker.stop()  # stop the face tracking thread
    tracking_thread.join(timeout=2)
    print("Stopped.")