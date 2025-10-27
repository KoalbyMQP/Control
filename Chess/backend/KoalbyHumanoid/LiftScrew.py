'''
Stores info on the screw lift, similar to Motor
'''

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from backend.KoalbyHumanoid.PID import PID

class Lift():
    def __init__(self, is_real, motor_id, name, M, lift_limit = None, serial = None, pidGains = None, sim = None, handle = None):
        self.is_real = is_real
        self.motor_id = motor_id
        self.name = name
        self.M = M
        self.prevTime = time.perf_counter()
        self.target = (0, 'P')
        
        if(is_real):
            self.lift_limit = lift_limit
            self.arduino_serial = serial
        else:
            self.sim = sim
            self.handle = handle
            self.simMovePID = PID(self.pidGains[0], self.pidGains[1], self.pidGains[2])
            
        self.height = None
        
    def get_position(self):
        raise NotImplementedError("Lift get pos not implemented")
        #TODO
        
    def set_position(self):
        raise NotImplementedError("Lift set pos not implemented")
        #TODO
        
    def get_velocity(self):
        raise NotImplementedError("Lift get vel not implemented")
        #TODO
        
    def set_velocity(self):
        raise NotImplementedError("Lift set vel not implemented")
        #TODO
        
    def moveTo(self, target = "TARGET"):
        targetPos = target[0]
        if self.is_real:
            targetPos = math.degrees(targetPos)
        
        if time.perf_counter() - self.prevTime > 0.001:
            if target == "TARGET":
                target = self.target
            if target[1] == 'P':
                self.set_position(targetPos)
            elif target[1] == 'V':
                self.set_velocity(targetPos)
            else:
                raise Exception("Invalid goal")
        self.prevTime = time.perf_counter()