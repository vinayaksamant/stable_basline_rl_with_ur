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

        # IK and FK clients
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.ik_client.wait_for_service()
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        self.fk_client.wait_for_service()

        # Trajectory publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Joint states subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_state = None
        self.get_logger().info("MoveURRobot node initialized")

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def get_current_cartesian_pose(self):
        while self.joint_state is None:
            rclpy.spin_once(self)
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = 'base_link'
        fk_req.robot_state.joint_state = self.joint_state
        fk_req.fk_link_names = ['wrist_3_link']
        future = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.pose_stamped:
            p = res.pose_stamped[0].pose
            return [
                p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
            ]
        else:
            self.get_logger().warn("Failed to get FK")
            return None

    def move_to_pose(self, target_pose):
        while self.joint_state is None:
            rclpy.spin_once(self)

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = 'ur_manipulator'
        ik_req.ik_request.robot_state.joint_state = self.joint_state
        ik_req.ik_request.pose_stamped = target_pose
        ik_req.ik_request.timeout.sec = 2
        ik_req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.error_code.val == res.error_code.SUCCESS:
            traj = JointTrajectory()
            traj.joint_names = res.solution.joint_state.name
            point = JointTrajectoryPoint()
            point.positions = res.solution.joint_state.position

            current_pose = self.get_current_cartesian_pose()
            if current_pose:
                distance = np.linalg.norm(
                    np.array([target_pose.pose.position.x,
                              target_pose.pose.position.y,
                              target_pose.pose.position.z]) - np.array(current_pose[:3])
                )
                speed = 0.1
                point.time_from_start.sec = max(2, int(distance / speed))
            else:
                point.time_from_start.sec = 2

            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            # Wait until pose is reached (with tolerance)
            for _ in range(100):
                current_pose = self.get_current_cartesian_pose()
                if current_pose:
                    error = np.linalg.norm(
                        np.array(current_pose[:3]) - 
                        np.array([target_pose.pose.position.x,
                                  target_pose.pose.position.y,
                                  target_pose.pose.position.z])
                    )
                    if error < 0.01:
                        break
                rclpy.spin_once(self)

            return True
        else:
            self.get_logger().warn("IK failed")
            return False
