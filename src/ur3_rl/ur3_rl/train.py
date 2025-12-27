# ur3_rl/train.py

from stable_baselines3 import PPO
from ur3_rl.ur3_env import UR3ReachAvoidEnv

env = UR3ReachAvoidEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=5)
model.save("ppo_ur3_reach_avoid")
