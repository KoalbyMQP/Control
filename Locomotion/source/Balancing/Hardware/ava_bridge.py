from ..Balance.balance_env_cfg import BalancingSceneCfg
robot = BalancingSceneCfg.robot

class ava_bridge:
    def connect(self):
        # connect to Ava
        pass


    def send_joint_positions(self, positions):
        # 1. Get current joint positions (in radians or meters)
        joint_positions = robot.data.joint_pos

        # 2. Get current joint velocities (in rad/s or m/s)
        joint_velocities = robot.data.joint_vel

        # 3. Get measured joint efforts / applied motor torques 
        joint_efforts = robot.data.measured_joint_efforts
        pass

    def read_joint_positions(self):
        # Read encoder positions

        pass

    def read_joint_velocities(self):
        # Read encoder velocities

        pass

    def read_imu(self):
        # Read IMU

        pass

    def emergency_stop(self):
        # Immediately disable motors

        pass



"""while simulation_running:




"""