import gym
import numpy as np
from gym import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from ur3e_rl.ur3e_motion import MoveURRobot

class UR3EEnv(gym.Env, Node):
    def __init__(self,goal=None, obstacle=None):
        rclpy.init(args=None)
        Node.__init__(self, 'ur3e_env_node')

        self.robot = MoveURRobot()

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.action_space = spaces.Box(low=-0.05, high=0.05, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        self.goal = np.array(goal) if goal is not None else np.array([0.1, 0.4, 0.6])
        self.obstacle = np.array(obstacle) if obstacle is not None else np.array([0.15, 0.3, 0.6])

        self.publish_marker(self.obstacle, 0, (1.0, 0.0, 0.0), 0.05, "obstacle")
        self.publish_marker(self.goal, 1, (0.0, 1.0, 0.0), 0.07, "goal")

        self.current_pos = None

    def publish_marker(self, position, marker_id, color, scale, ns):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.marker_pub.publish(marker)

    def reset(self):
        home_pose = self.create_pose([0.0, 0.2, 0.7, 0.1, 0.0, 0.0, 0.0])
        success = self.robot.move_to_pose(home_pose)
        if not success:
            self.get_logger().warn("Failed to move to home pose on reset")

        pos = self.robot.get_current_cartesian_pose()
        if pos:
            self.current_pos = np.array(pos[:3])
        else:
            self.current_pos = np.zeros(3)
        return self.current_pos

    def step(self, action):
        next_pos = self.current_pos + action
        target_pose = self.create_pose([
            next_pos[0], next_pos[1], next_pos[2],
            -0.7, 0.0, 0.0, 0.7
        ])

        ik_success = self.robot.move_to_pose(target_pose)
        if not ik_success:
            reward = -10.0
            done = True
            return self.current_pos, reward, done, {}

        pos = self.robot.get_current_cartesian_pose()
        if pos:
            self.current_pos = np.array(pos[:3])

        dist_goal = np.linalg.norm(self.goal - self.current_pos)
        dist_obstacle = np.linalg.norm(self.obstacle - self.current_pos)

        reward = -dist_goal
        if dist_obstacle < 0.05:
            reward -= 10.0

        done = dist_goal < 0.02

        return self.current_pos, reward, done, {}

    def create_pose(self, pose_list):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = pose_list[0]
        pose.pose.position.y = pose_list[1]
        pose.pose.position.z = pose_list[2]
        pose.pose.orientation.x = pose_list[3]
        pose.pose.orientation.y = pose_list[4]
        pose.pose.orientation.z = pose_list[5]
        pose.pose.orientation.w = pose_list[6]
        return pose

    def close(self):
        self.robot.destroy_node()
        self.destroy_node()
        rclpy.shutdown()
