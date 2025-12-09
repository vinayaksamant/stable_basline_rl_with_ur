#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class MoveURRobot(Node):
    def __init__(self):
        super().__init__('move_ur_robot_node')
        
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.ik_client.wait_for_service()
        
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        self.fk_client.wait_for_service()

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_state = None
        self.get_logger().info("MoveURRobot node has been initialized")

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def get_current_cartesian_pose(self):
        while self.joint_state is None:
            rclpy.spin_once(self)

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = 'base_link'
        fk_request.robot_state.joint_state = self.joint_state
        fk_request.fk_link_names = ['wrist_3_link']

        future = self.fk_client.call_async(fk_request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response and response.pose_stamped:
            pose = response.pose_stamped[0].pose
            return [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
        else:
            self.get_logger().warn("Failed to get forward kinematics")
            return None

    def move_to_pose(self, target_pose):
        while self.joint_state is None:
            rclpy.spin_once(self)

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = 'ur_manipulator'
        ik_request.ik_request.robot_state.joint_state = self.joint_state
        ik_request.ik_request.pose_stamped = target_pose
        ik_request.ik_request.timeout.sec = 2
        ik_request.ik_request.avoid_collisions = False

        future = self.ik_client.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.error_code.val == response.error_code.SUCCESS:
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = response.solution.joint_state.name

            point = JointTrajectoryPoint()
            point.positions = response.solution.joint_state.position

            # Compute distance for time scaling
            current_pose = self.get_current_cartesian_pose()
            if current_pose:
                distance = np.linalg.norm(
                    np.array([target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z])
                    - np.array(current_pose[:3])
                )
                speed = 0.1
                time_required = max(2, distance / speed)
                point.time_from_start.sec = int(time_required)
            else:
                point.time_from_start.sec = 2

            trajectory_msg.points.append(point)

            self.trajectory_publisher.publish(trajectory_msg)
            self.get_logger().info("Trajectory published to move the robot")

            # Wait until close enough
            for _ in range(100):
                current_pose = self.get_current_cartesian_pose()
                if current_pose:
                    error = np.linalg.norm(
                        np.array(current_pose[:3]) -
                        np.array([
                            target_pose.pose.position.x,
                            target_pose.pose.position.y,
                            target_pose.pose.position.z
                        ])
                    )
                    if error < 0.01:
                        break
                rclpy.spin_once(self)

            self.get_logger().info("Movement done")
            return True
        else:
            self.get_logger().warn("Failed to find an inverse kinematics solution")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MoveURRobot()
    
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = 0.2
    target_pose.pose.position.z = 0.7
    target_pose.pose.orientation.x = -0.7
    target_pose.pose.orientation.y = 0.2
    target_pose.pose.orientation.z = 0.6
    target_pose.pose.orientation.w = 0.7
 
    # node.move_to_pose(target_pose) 
    node.get_current_cartesian_pose()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# Cartesain coordinates of home [0.0, 0.2, 0.7, -0.7, 0.0, 0.0, 0.7]
# 0.1, 0.4, 0.6, 0.7, 0.2, 0.6, -0.4]




