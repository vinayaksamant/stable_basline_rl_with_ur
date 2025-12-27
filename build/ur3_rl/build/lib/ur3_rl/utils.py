# ur3_rl/utils.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import numpy as np

class UR3Controller(Node):
    def __init__(self):
        super().__init__('ur3_controller')
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.current_pose = None
        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_current_pose(self):
        while self.current_pose is None:
            rclpy.spin_once(self)
        return self.current_pose

    def move_to_delta(self, dx, dy, dz):
        pose = self.get_current_pose()
        new_pose = PoseStamped()
        new_pose.header.frame_id = 'base_link'
        new_pose.pose.position.x = pose.pose.position.x + dx
        new_pose.pose.position.y = pose.pose.position.y + dy
        new_pose.pose.position.z = pose.pose.position.z + dz
        new_pose.pose.orientation = pose.pose.orientation  # Keep same orientation
        self.pose_pub.publish(new_pose)
        self.get_logger().info(f"Moving to delta: ({dx}, {dy}, {dz})")
        rclpy.spin_once(self, timeout_sec=2.0)
