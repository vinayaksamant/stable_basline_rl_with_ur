#!/usr/bin/env python3
'''
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, ExecuteKnownTrajectory

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        # Adjusted to use /move_group for execution if /execute_kinematic_path is unavailable
        self.execute_client = self.create_client(ExecuteKnownTrajectory, '/move_group')

    def plan_and_execute(self):
        # Call /compute_ik
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.pose_stamped.header.frame_id = "base_link"
        ik_request.ik_request.pose_stamped.pose.position.x = 0.4
        ik_request.ik_request.pose_stamped.pose.position.y = 0.2
        ik_request.ik_request.pose_stamped.pose.position.z = 0.5
        ik_request.ik_request.pose_stamped.pose.orientation.w = 1.0
        
        self.get_logger().info("Waiting for /compute_ik service...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available')
            return
        
        self.get_logger().info("Requesting IK solution...")
        try:
            ik_response = self.ik_client.call(ik_request)
            if ik_response.error_code.val != ik_response.error_code.SUCCESS:
                self.get_logger().error('IK computation failed')
                return
            self.get_logger().info(f"IK Response: {ik_response.solution.joint_state.position}")
        except Exception as e:
            self.get_logger().error(f"Error calling IK service: {e}")
            return

        # Call /plan_kinematic_path
        plan_request = GetMotionPlan.Request()
        plan_request.request.group_name = "arm"
        plan_request.request.goal_constraints = [
            # Set joint constraints or pose goals here
        ]
        
        self.get_logger().info("Waiting for /plan_kinematic_path service...")
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Planning service not available')
            return
        
        self.get_logger().info("Planning motion...")
        try:
            plan_response = self.plan_client.call(plan_request)
            self.get_logger().info(f"Motion Plan: {plan_response.motion_plan_response}")
        except Exception as e:
            self.get_logger().error(f"Error calling planning service: {e}")
            return

        # Call /move_group (or /execute_kinematic_path if available)
        execute_request = ExecuteKnownTrajectory.Request()
        execute_request.trajectory = plan_response.motion_plan_response.trajectory
        
        self.get_logger().info("Waiting for /move_group service...")
        if not self.execute_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Execution service not available')
            return
        
        self.get_logger().info("Executing trajectory...")
        try:
            execute_response = self.execute_client.call(execute_request)
            self.get_logger().info(f"Execution Result: {execute_response.error_code}")
        except Exception as e:
            self.get_logger().error(f"Error calling execution service: {e}")
            return

def main():
    rclpy.init()
    planner = MotionPlanner()
    planner.plan_and_execute()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

import rclpy
from rclpy.node import Node
from moveit2 import MoveGroupInterface

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.move_group = MoveGroupInterface(node=self, group_name='manipulator')

    def move_to_pose(self, pose):
        self.move_group.set_pose_target(pose)
        plan = self.move_group.plan()
        if plan:
            self.move_group.execute(plan)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()

    # Define your target pose here
    target_pose = {
        'position': {'x': 0.5, 'y': 0.0, 'z': 0.5},
        'orientation': {'x': 0.0, 'y': 1.0, 'z': 0.0, 'w': 0.0}
    }

    node.move_to_pose(target_pose)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

