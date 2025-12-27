#!/usr/bin/env python3

from stable_baselines3 import PPO
from ur3e_rl.ur3e_env import UR3EEnv

def main():
    env = UR3EEnv()
    model = PPO.load("ur3e_rl_model")

    # Example new goal for testing
    new_goal = [0.2, 0.3, 0.6]
    obs = env.reset(goal=new_goal)

    done = False
    step_count = 0
    max_steps = 50

    while not done and step_count < max_steps:
        action, _ = model.predict(obs)
        obs, reward, done, info = env.step(action)
        print(f"Obs: {obs}, Reward: {reward}")
        step_count += 1

    env.close()

if __name__ == '__main__':
    main()
