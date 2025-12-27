# ur3_rl/ur3_env.py

import gym
from gym import spaces
import numpy as np
import rclpy
from ur3_rl.utils import UR3Controller

class UR3ReachAvoidEnv(gym.Env):
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = UR3Controller()

        # Define goal and obstacle (fixed for now)
        self.goal = np.array([0.4, 0.1, 0.2])
        self.obstacle = np.array([0.3, 0.0, 0.2])
        self.robot_position = np.zeros(3)

        # Observation: current + goal + obstacle
        self.observation_space = spaces.Box(
            low=-2.0, high=2.0, shape=(9,), dtype=np.float32
        )

        # Action: delta x, y, z
        self.action_space = spaces.Box(
            low=-0.01, high=0.01, shape=(3,), dtype=np.float32
        )

    def reset(self):
        self.robot_position = np.array([0.3, -0.1, 0.2])
        dx = self.robot_position - self.node.get_current_pose().pose.position[:3]
        self.node.move_to_delta(*dx)
        obs = np.concatenate([self.robot_position, self.goal, self.obstacle])
        return obs.astype(np.float32)

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.node.move_to_delta(*action)
        pose = self.node.get_current_pose()
        self.robot_position = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ])

        done = False
        reward = -np.linalg.norm(self.goal - self.robot_position)

        if self._check_collision():
            reward = -100
            done = True

        if np.linalg.norm(self.goal - self.robot_position) < 0.01:
            reward = 100
            done = True

        obs = np.concatenate([self.robot_position, self.goal, self.obstacle])
        return obs.astype(np.float32), reward, done, {}

    def _check_collision(self):
        dist = np.linalg.norm(self.robot_position - self.obstacle)
        return dist < 0.05

    def close(self):
        rclpy.shutdown()
