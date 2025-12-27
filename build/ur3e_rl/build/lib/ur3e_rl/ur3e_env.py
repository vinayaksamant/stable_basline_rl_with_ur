import gym
import numpy as np
from gym import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from ur3e_rl.ur3e_motion import MoveURRobot

class UR3EEnv(gym.Env, Node):
    def __init__(self, base_height=0.2):
        rclpy.init(args=None)
        Node.__init__(self, 'ur3e_env_node')

        self.robot = MoveURRobot()
        self.base_height = base_height

        # Action: delta x, y, z
        self.action_space = spaces.Box(low=-0.05, high=0.05, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Default goal
        self.goal = np.array([0.1, 0.4, 0.6])

        # Planning scene publisher
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.add_sphere_obstacle()

        self.current_pos = None

    def add_sphere_obstacle(self, position=[0.25, 0.0, 0.45], radius=0.08):
        scene = PlanningScene()
        scene.is_diff = True
        co = CollisionObject()
        co.id = "rl_sphere_obstacle"
        co.header.frame_id = "base_link"

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [radius]

        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0

        co.primitives.append(sp)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene.world.collision_objects.append(co)
        self.scene_pub.publish(scene)
        self.get_logger().info("Sphere obstacle added")

    def set_goal(self, goal):
        self.goal = np.array(goal)

    def reset(self, goal=None):
        if goal is not None:
            self.set_goal(goal)
        # Move to home
        home_pose = self.create_pose([0.0, 0.2, 0.7, -0.7, 0.0, 0.0, 0.7])
        success = self.robot.move_to_pose(home_pose)
        if not success:
            self.get_logger().warn("Failed to move to home pose on reset")

        pos = self.robot.get_current_cartesian_pose()
        self.current_pos = np.array(pos[:3]) if pos else np.zeros(3)
        return self.current_pos

    def step(self, action):
        next_pos = self.current_pos + action

        # enforce base height
        next_pos[2] = max(next_pos[2], self.base_height)

        target_pose = self.create_pose([next_pos[0], next_pos[1], next_pos[2],
                                        -0.7, 0.0, 0.0, 0.7])
        ik_success = self.robot.move_to_pose(target_pose)

        if not ik_success:
            reward = -10.0
            done = True
            return self.current_pos, reward, done, {}

        pos = self.robot.get_current_cartesian_pose()
        if pos:
            self.current_pos = np.array(pos[:3])

        dist_goal = np.linalg.norm(self.goal - self.current_pos)
        dist_obstacle = np.linalg.norm(np.array([0.25, 0.0, 0.45]) - self.current_pos)
        reward = -dist_goal
        if dist_obstacle < 0.05:
            reward -= 10.0

        done = dist_goal < 0.02
        return self.current_pos, reward, done, {}

    def create_pose(self, pose_list):
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = pose_list[:3]
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = pose_list[3:]
        return p

    def close(self):
        self.robot.destroy_node()
        self.destroy_node()
        rclpy.shutdown()
