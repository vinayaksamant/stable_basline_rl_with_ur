#!/usr/bin/env python3

import gym
from stable_baselines3 import PPO
from ur3e_rl.ur3e_env import UR3EEnv


def main():
    env = UR3EEnv()

    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=5)
    model.save("ur3e_rl_model")

    env.close()

if __name__ == '__main__':
    main()