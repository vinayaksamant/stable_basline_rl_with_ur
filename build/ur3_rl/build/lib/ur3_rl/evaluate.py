# ur3_rl/evaluate.py

from stable_baselines3 import PPO
from ur3_rl.ur3_env import UR3ReachAvoidEnv
import time

env = UR3ReachAvoidEnv()
model = PPO.load("ppo_ur3_reach_avoid")

obs = env.reset()
done = False
while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    time.sleep(1.0)
