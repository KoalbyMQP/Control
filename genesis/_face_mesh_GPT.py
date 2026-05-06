# https://mediapipe.readthedocs.io/en/latest/solutions/face_mesh.html
# https://docs.luxonis.com/software-v3/depthai/examples/image_align/depth_align
# https://docs.luxonis.com/software-v3/depthai/examples/spatial_location_calculator/spatial_location_calculator/
"""
Scaffolding generated with minimal input from GPT-4.1 from face_mesh.py and obj_tracker.py (see `conversation.md`)

wrapped in a class for outside usage 
"""
import cv2 # from OpenCV Python lib
import depthai as dai # Luxonis DepthAI v3.0.0 -- TODO: migrate to Depthai ROS when possible
import mediapipe as mp # MediaPipe v0.10.21
import numpy as np
import time
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_face_mesh = mp.solutions.face_mesh


class faceTracker:
    def __init__(self):
        self.colour = (255, 255, 255)

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        rgb_in = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A) # RGB camera
        left_in = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B) # left IR camera
        right_in = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C) # right IR camera

        # Nodes
        stereo = self.pipeline.create(dai.node.StereoDepth) # for linking depth
        spatial = self.pipeline.create(dai.node.SpatialLocationCalculator) # for spatial calculations
        sync = self.pipeline.create(dai.node.Sync) # for colour camera
        # Inputs
        rgb_out = rgb_in.requestOutput((1280, 720), enableUndistortion=True)
        left_out = left_in.requestOutput((640, 480))
        right_out = right_in.requestOutput((640, 480))
        # Linking
        rgb_out.link(sync.inputs['RGB'])
        left_out.link(stereo.left)
        right_out.link(stereo.right)

        stereo.setRectification(True)
        stereo.setExtendedDisparity(True)

        self.config = dai.SpatialLocationCalculatorConfigData()
        self.config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MODE
        self.config.depthThresholds.lowerThreshold = 10
        self.config.depthThresholds.upperThreshold = 10000
        self.config.roi = dai.Rect(0,0,0,0) # define region of interest (ROI) for spatio-depth calculations

        spatial.inputConfig.setWaitForMessage(False)
        spatial.initialConfig.addROI(self.config)

        self.spatial_queue = spatial.out.createOutputQueue(maxSize=4, blocking=False)
        self.depth_queue = spatial.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)
        self.rgb_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)

        stereo.depth.link(spatial.inputDepth)
        rgb_out.link(stereo.inputAlignTo)

        self.inputConfigQueue = spatial.inputConfig.createInputQueue()

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)

        self.mouse_coords=[0,0]
        self.old_mouse_coords=self.mouse_coords[:]
        
        self.windowName = "OAK-D Lite Face Mesh"
        self.depthWeight = 0
        self.colourWeight = 1
        self.show_face_mesh = True
        self.filepath = 'face_mesh_GPT.png'

        self.forehead_pos = None

        self.run_flag = False

    def mouse_callback(self, event,x,y,flags,param):
        self.mouse_coords[0]=x
        self.mouse_coords[1]=y
    
    def get_forehead_pos(self):
        return self.forehead_pos

    def rref(self, a:np.ndarray) -> np.ndarray:
        A=a.astype(np.float64)
        h=0
        k=0
        while ((h<len(A))&(k<len(A[0]))):
            i_max=np.argmax(abs(A.T[k][h:]))+h
            if (A[i_max,k]==0):
                k+=1
            else:
                t=1*A[i_max]
                A[i_max]=1*A[h]
                A[h]=t
                for i in range(len(A)):
                    if (i!=h):
                        f=A[i,k]/A[h,k]
                        A[i,k:]-=A[h,k:]*f
                A[h]/=A[h,k]
                h+=1
                k+=1
        return A

    def stop(self):
        self.run_flag = False
        try:
            self.pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()

    def run(self):
        self.run_flag = True
        with self.pipeline:
            self.pipeline.start()
            cv2.namedWindow(self.windowName, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.windowName, 1280, 720)
            drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
            face_mesh = mp_face_mesh.FaceMesh(
                max_num_faces=10, # can only recognize up to 10 human faces at a time
                refine_landmarks=True,
                min_detection_confidence=0.5, # may lead to false positives (artifacts) if max_num_faces > 1
                min_tracking_confidence=0.5
            ) # 'with-as' block is inefficient for streams
            while (self.pipeline.isRunning() and self.run_flag):
                rgb_data = self.rgb_queue.tryGet()
                if rgb_data is None:
                    if not self.run_flag:
                        break
                    time.sleep(0.001)
                    continue

                spatial_msg = self.spatial_queue.tryGet()
                spatial_data = spatial_msg.getSpatialLocations() if spatial_msg is not None else []

                outputDepthImage : dai.ImgFrame = self.depth_queue.tryGet()
                if outputDepthImage is None:
                    if not self.run_flag:
                        break
                    time.sleep(0.001)
                    continue

                frameDepth = outputDepthImage.getFrame() # getCvFrame is technically slower but handles overhead
                cvFrame = rgb_data['RGB'].getCvFrame()

                depthFrameColour = cv2.normalize(frameDepth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                depthFrameColour = cv2.equalizeHist(depthFrameColour)
                depthFrameColour = cv2.applyColorMap(-depthFrameColour, cv2.COLORMAP_INFERNO)

                blended = cv2.addWeighted(
                    cvFrame, self.colourWeight, depthFrameColour, self.depthWeight, 0
                )
                height, width, _ = cvFrame.shape

                if len(spatial_data):
                    depth_data_cursor = spatial_data[0] # first ROI is guaranteed to be that of the cursor
                    roi = depth_data_cursor.config.roi
                    fontType = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(blended, f"x: {int(depth_data_cursor.spatialCoordinates.x)} mm", (self.mouse_coords[0] + 10, self.mouse_coords[1] + 20), fontType, 0.3, self.colour, 1)
                    cv2.putText(blended, f"y: {int(depth_data_cursor.spatialCoordinates.y)} mm", (self.mouse_coords[0] + 10, self.mouse_coords[1] + 35), fontType, 0.3, self.colour, 1)
                    cv2.putText(blended, f"z: {int(depth_data_cursor.spatialCoordinates.z)} mm", (self.mouse_coords[0] + 10, self.mouse_coords[1] + 50), fontType, 0.3, self.colour, 1)

                results = face_mesh.process(cv2.cvtColor(cvFrame, cv2.COLOR_BGR2RGB))
                # frame_out = blended.copy() # using `blended` from this point onwards may be reckless
                if (results.multi_face_landmarks!=None) & self.show_face_mesh:
                    # Key landmark indices and labels
                    landmark_labels = {
                        1: "Nose Tip",
                        33: "Right Eye",
                        263: "Left Eye",
                        61: "Mouth",
                        151: "Forehead",
                        337: "A1",
                        10: "A2",
                        108: "A3"
                    }
                    forehead_labels = {
                        i:"Forehead" for i in [162,71,63,105,66,107,109,67,103,54,21,108,69,104,68,389,301,293,334,296,336,9,151,10,338,297,332,284,251,337,299,333,298]
                    }
                    configs = [self.config]
                    landmarks_list = []
                    for face_landmarks in results.multi_face_landmarks:
                        mp_drawing.draw_landmarks(
                            image=blended,
                            landmark_list=face_landmarks,
                            connections=mp_face_mesh.FACEMESH_TESSELATION,
                            landmark_drawing_spec=None,
                            connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_tesselation_style())
                        mp_drawing.draw_landmarks(
                            image=blended,
                            landmark_list=face_landmarks,
                            connections=mp_face_mesh.FACEMESH_CONTOURS,
                            landmark_drawing_spec=None,
                            connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_contours_style())
                        mp_drawing.draw_landmarks(
                            image=blended,
                            landmark_list=face_landmarks,
                            connections=mp_face_mesh.FACEMESH_IRISES,
                            landmark_drawing_spec=None,
                            connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_iris_connections_style())

                        # Get coordinates of landmarks
                        for idx, lm in enumerate(face_landmarks.landmark):
                            if idx in landmark_labels:
                                config_lm = dai.SpatialLocationCalculatorConfigData()
                                config_lm.depthThresholds.lowerThreshold = self.config.depthThresholds.lowerThreshold
                                config_lm.depthThresholds.upperThreshold = self.config.depthThresholds.upperThreshold
                                config_lm.roi = dai.Rect(int(width*lm.x),int(height*lm.y),2,2)
                                configs.append(config_lm)
                                landmarks_list.append([idx, lm])
                            # Draw forehead dots
                            if idx in forehead_labels:
                                cv2.circle(blended, (int(width*lm.x), int(height*lm.y)), 2, (255, 0, 0), -1)

                    if len(configs):
                        cfg = dai.SpatialLocationCalculatorConfig()
                        cfg.setROIs(configs)
                        self.inputConfigQueue.send(cfg)
                    for depth_data in spatial_data[1:]:
                        try:landmark = landmarks_list.pop()
                        except IndexError:continue
                        
                        x_px = int(width*landmark[1].x); y_px = int(height*landmark[1].y) # whether one rounds or floors is inconsequential
                        
                        if (landmark_labels[landmark[0]] in ("A1","A2","A3")):
                            if (landmark_labels[landmark[0]]=="A1"):
                                u = np.array([[depth_data.spatialCoordinates.x],[depth_data.spatialCoordinates.y],[depth_data.spatialCoordinates.z]])
                                P = np.array([[x_px],[y_px]])
                            elif (landmark_labels[landmark[0]]=="A2"):
                                v = np.array([[depth_data.spatialCoordinates.x],[depth_data.spatialCoordinates.y],[depth_data.spatialCoordinates.z]])
                                Q = np.array([[x_px],[y_px]])
                            elif (landmark_labels[landmark[0]]=="A3"):
                                w_coords = np.array([[depth_data.spatialCoordinates.x],[depth_data.spatialCoordinates.y],[depth_data.spatialCoordinates.z]])
                                R = np.array([[x_px],[y_px]])
                        else:
                            if (landmark_labels[landmark[0]]=="Forehead"):
                                try:
                                    # Check all required variables exist and are valid
                                    if not all([hasattr(locals().get(k), '__class__') for k in ['u','v','w_coords','P','Q','R']]):
                                        raise NameError("Missing coordinate variables")
                                    if any(np.isnan(np.concatenate([u.flatten(), v.flatten(), w_coords.flatten(), P.flatten(), Q.flatten(), R.flatten()]))):
                                        raise ValueError("NaN in coordinate data")
                                    A = self.rref(np.vstack((np.vstack((u.T,v.T,w_coords.T)).T,np.vstack((P.T,Q.T,R.T)).T)).T).T[3:]
                                    forehead_normal = np.cross(v-u,w_coords-v,axis=0)
                                    forehead_normal = A@forehead_normal/np.linalg.norm(forehead_normal)*5+np.array([[x_px],[y_px]])
                                    self.forehead_pos = (int(depth_data.spatialCoordinates.x), int(depth_data.spatialCoordinates.y), int(depth_data.spatialCoordinates.z))
                                    cv2.putText(blended, f"NORM", (int(forehead_normal[0,0]), int(forehead_normal[1,0])), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)
                                except (NameError, ValueError, KeyError):continue
                            cv2.putText(blended, f"{landmark_labels[landmark[0]]}", (x_px, y_px-15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)
                            cv2.putText(blended, f"x: {int(depth_data.spatialCoordinates.x)} mm", (x_px, y_px-5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)
                            cv2.putText(blended, f"y: {int(depth_data.spatialCoordinates.y)} mm", (x_px, y_px+5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)
                            cv2.putText(blended, f"z: {int(depth_data.spatialCoordinates.z)} mm", (x_px, y_px+15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)

                # Show the frame
                cv2.imshow(self.windowName, blended)
                cv2.setMouseCallback(self.windowName, self.mouse_callback)
                if self.old_mouse_coords != self.mouse_coords:
                    self.old_mouse_coords = self.mouse_coords[:]
                    self.config.roi = dai.Rect(self.mouse_coords[0]-1,self.mouse_coords[1]-1,2,2)
                    cfg = dai.SpatialLocationCalculatorConfig()
                    cfg.addROI(self.config)
                    self.inputConfigQueue.send(cfg)

                key = cv2.waitKey(1)
                if key & 0xFF == ord('1'): # press '1' to switch to depth view
                    self.depthWeight=1;self.colourWeight=0
                elif key & 0xFF == ord('2'): # press '2' to switch to combined view
                    self.depthWeight=0.5;self.colourWeight=0.5
                elif key & 0xFF == ord('3'): # press '3' to switch to colour view
                    self.depthWeight=0;self.colourWeight=1
                elif key & 0xFF == ord('m'): # press 'M' to toggle face mesh
                    self.show_face_mesh = False if self.show_face_mesh else True
                elif key & 0xFF == ord('s'): # press 'S' to save screenshot
                    cv2.imwrite(self.filepath, blended)
                elif key & 0xFF in (ord('q'),27): # press 'Q' or 'ESC' to exit window
                    break
            del face_mesh
    
if __name__ == "__main__":
    tracker = faceTracker()
    tracker.run()
    # time.sleep(1)
    # print("stopping")
    # tracker.stop()
    # print(tracker.get_forehead_pos())