import torch
import numpy as np
from imu_terms import synthetic_imu
from avalocomotion_env_cfg import AvaLocomotionEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

env_cfg = AvaLocomotionEnvCfg()
env = ManagerBasedRLEnv(cfg=env_cfg)

env.reset()

imu_log = []
actions = torch.zeros_like(env.action)

for _ in range(500):
	env.step(actions)
	imu = synthetic_imu(env)
	imu_log.append(imu[0].cpu().numpy())


np.savetxt("imu_test.csv", np.array(imu_log), delimiter=",")
print("Saved imu_test.csv")
