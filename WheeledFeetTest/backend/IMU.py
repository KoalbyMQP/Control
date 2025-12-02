import adafruit_bno055
import numpy as np
import math

try:
    import board
except NotImplementedError:
    print("Failed to import board when not running on Raspberry Pi")

class IMU():
    def __init__(self, isReal, sim, imu_name):
        """
        Initialize the IMU.

        Parameters:
        - isReal: Boolean indicating if the IMU is real or simulated.
        - sim: The simulation client for accessing simulation signals.
        - imu_name: The name of the IMU (e.g., "RightFoot", "LeftFoot") to differentiate multiple IMUs in simulation.
        """
        self.isReal = isReal
        self.isConnected = True
        self.sim = sim
        self.imu_name = imu_name  # Name for identifying multiple IMUs in simulation

        if isReal:
            try:
                i2c = board.I2C()  # uses board.SCL and board.SDA
                self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            except:
                print("No IMU detected, disabling IMU")
                self.isConnected = False
        else:
            if self.sim is None:
                raise ValueError("Simulation client must be provided for simulated IMU")
            else:
                self.isConnected = True

    def zero(self):
        if self.isReal:
            self.zeroAngles = self.getDataRaw()
        else:
            raise NotImplementedError("IMU zero not implemented for simulation IMU")

    def getData(self):
        return self.getDataRaw()

    # x axis is toward Ava's Left, y axis is up, z axis is toward Ava's front, all from the center of Ava
    # getData returns [x angle (rad), y angle (rad), z angle (rad), x acceleration (m/s^2), y acceleration (m/s^2), z acceleration (m/s^2)]
    def getDataRaw(self): 
        if not self.isConnected:
            return [0, 0, 0, 0, 0, 0]
        if self.isReal:
            # In terms of right-hand rule convention for positive directions:
            # sensor.euler returns: (yaw (opposite convention), roll (normal convention), pitch (opposite convention)
            # yaw is in range 0 to 360, roll is -90 to 90, pitch is -180 to 180
            yaw, roll, pitch = self.sensor.euler
            
            self.data = [ 
                math.radians(-pitch),
                math.radians(-(yaw if yaw <= 180 else yaw - 360)),  # Mapping from 0 to 360 to -180 to 180
                math.radians(roll),
                self.sensor.acceleration[0],
                -self.sensor.acceleration[2],
                self.sensor.acceleration[1]
            ]
        elif self.sim:
            # Different signal names for each IMU based on imu_name
            prefix = f"imu{self.imu_name}"
            self.data = [
                self.sim.getFloatSignal(f"{prefix}_gyroX"),
                self.sim.getFloatSignal(f"{prefix}_gyroY"),
                self.sim.getFloatSignal(f"{prefix}_gyroZ"),
                self.sim.getFloatSignal(f"{prefix}_accelX"),
                self.sim.getFloatSignal(f"{prefix}_accelY"),
                self.sim.getFloatSignal(f"{prefix}_accelZ"),
            ]
            # Handle cases where signals may not be available (default to 0 if no data)
            self.data = [0 if dataPoint is None else dataPoint for dataPoint in self.data]
        else:
            self.data = [0, 0, 0, 0, 0, 0]  # Fallback if sim is not set in simulation mode
        return self.data

class IMUManager():
    def __init__(self, isReal, sim):
        """
        Initialize the IMU manager for handling multiple IMUs.
        
        Parameters:
        - isReal: Boolean indicating if the IMUs are real or simulated.
        - sim: The simulation client for accessing simulation signals.
        """
        self.isReal = isReal
        self.sim = sim
        self.imu_names = ["Torso"]
        self.imus = {name: IMU(isReal, sim, name) for name in self.imu_names}

    def getAllIMUData(self):
        """
        Retrieve data for all IMUs.
        
        Returns:
        - A dictionary where keys are IMU names and values are the IMU data.
        """
        imu_data = {name: self.imus[name].getData() for name in self.imu_names}
        return imu_data