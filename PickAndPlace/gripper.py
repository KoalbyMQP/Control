
# ---------------- Imports ---------------------
'''
    Import from IKPY Modules and other 
    necessary libraries (maybe OMPL)
'''
import pandas as pd       # to read in information from the gripper data file
import numpy as np        # to handle some of the complex math

# ------------- Class Definition --------------

class Gripper():
    '''
        The Gripper class is for the pick and place can be scaled
        to different grippers more easily

        Properties:
    '''

    def __init__(self, type):
        self = self
        self.type = type
        self.status = False
        self.length = None
        self.rot_axis = None
        
        self.setGripperInfo()

    def setGripperInfo(self):
        '''
            Set length and rot_axis of gripper based on manually created
            DB or file (file is hardcoded)
        '''
        data_file = "gripper_info.csv"
        gripper_data = pd.read_csv(data_file).set_index("type")

        # set all of the gripper values
        gripper_info = gripper_data.loc[self.type]
        self.length = gripper_info['length']
        self.rot_axis = gripper_info['rot_axis']
    
    def getGripperInfo(self):
        '''
            Return the info of the gripper
        '''
        type = self.type
        length = self.length
        rot_axis = self.rot_axis
        status = self.status

        return type, length, rot_axis, status