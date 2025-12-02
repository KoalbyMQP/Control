import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from backend.SimpleBot.PID import PID

class Motor():
    def __init__(self, is_real, motor_id, name, twist, M, angle_limit=None,
                 serial=None, pidGains=None, sim=None, handle=None):
        
        self.is_real = is_real
        self.motor_id = motor_id
        self.name = name
        self.twist = twist
        self.M = M
        self.prevTime = time.perf_counter()
        self.target = (0, 'P')
        self.theta = None

        if is_real:
            self.angle_limit = angle_limit
            self.arduino_serial = serial

        else:
            self.pidGains = pidGains
            self.sim = sim
            self.handle = handle

            # PID for torque-based position control
            self.simMovePID = PID(*pidGains)

            # move out of set_position() !
            self.maxTorque = 10.0   # Nm

    # --------------------------------------------------

    def get_position(self):
        if self.is_real:
            self.arduino_serial.send_command(f"5 {self.motor_id}")
            current_position = self.arduino_serial.read_float()

            # IMPORTANT: convert to radians if Arduino sends degrees
            current_position = math.radians(current_position)

        else:
            current_position = self.sim.getJointPosition(self.handle)

        return current_position

    # --------------------------------------------------

    def set_position(self, position, time=1000):
        if self.is_real:
            self.arduino_serial.send_command(f"10 {self.motor_id} {position} {time}")
            return

        # ---------------- SIM TORQUE MODE PID ----------------
        
        self.theta = self.get_position()
        error = position - self.theta

        # PID update
        self.simMovePID.setError(error)
        torque = self.simMovePID.calculate()

        # clamp torque output
        torque = max(-self.maxTorque, min(self.maxTorque, torque))

        # ensure force mode
        self.sim.setJointMode(self.handle, self.sim.jointmode_force, 0)

        # send torque
        self.sim.setJointTargetForce(self.handle, torque)

        # required: apply a small “motion direction”
        if abs(error) > 0.001:
            vel = 10 if error > 0 else -10
        else:
            vel = 0
        self.sim.setJointTargetVelocity(self.handle, vel)

    # --------------------------------------------------

    def set_torque_real(self, on):
        if self.is_real:
            self.arduino_serial.send_command(f"20 {self.motor_id} {int(on)}")
            return
        else:
            raise Exception("set_torque_real called on simulated motor")
        
    def enable_torque_mode_sim(self):
        if self.is_real:
            raise Exception("Not for real motor")
        self.sim.setJointMode(self.handle, self.sim.jointmode_force, 0)
        self.sim.setJointTargetVelocity(self.handle, 0)

    def set_torque_sim(self, torque):
        if self.is_real:
            raise Exception("set_torque_sim called on real motor")

        # ensure force mode
        self.sim.setJointMode(self.handle, self.sim.jointmode_force, 0)

        # send torque
        self.sim.setJointTargetForce(self.handle, torque)

    # --------------------------------------------------

    def get_velocity(self):
        raise NotImplementedError("get_velocity in Motor not implemented")

    def set_velocity(self, velocity):
        if self.is_real:
            self.arduino_serial.send_command(f"40 {self.motor_id} {velocity}")
        else:
            self.sim.setJointTargetVelocity(self.handle, velocity)

    # --------------------------------------------------

    def move(self, target="TARGET"):
        # typical target tuple = (angle, 'P') or (velocity, 'V')

        # new target supplied
        if target != "TARGET":
            # reset integrator when target is replaced
            if hasattr(self, "simMovePID"):
                self.simMovePID.clear()
            self.target = target

        targetPos = self.target[0]
        goalType = self.target[1]

        if self.is_real and goalType == 'P':
            targetPos = math.degrees(targetPos)

        # 1 kHz update loop
        if time.perf_counter() - self.prevTime > 0.001:
            if goalType == 'P':
                self.set_position(targetPos)
            elif goalType == 'V':
                self.set_velocity(targetPos)
            else:
                raise Exception("Invalid goal")
            self.prevTime = time.perf_counter()
