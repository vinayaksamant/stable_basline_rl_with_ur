#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
import moveit_commander

class UR3eMotion(Node):
    def __init__(self):
        super().__init__('ur3e_motion')

        # Initialize MoveIt Commander
        moveit_commander.roscpp_initialize([])  # Initialize MoveIt 2
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander('manipulator')  # 'manipulator' is the planning group name for UR3e

        # Configure planning parameters
        self.group.set_planning_time(10.0)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

        self.get_logger().info('MoveIt Commander initialized and ready!')

    def move_to_pose(self, target_pose):
        self.group.set_pose_target(target_pose)

        # Plan the trajectory
        self.get_logger().info('Planning motion...')
        plan = self.group.plan()

        if plan:
            self.get_logger().info('Plan successful! Executing motion...')
            success = self.group.execute(plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            if success:
                self.get_logger().info('Motion executed successfully!')
            else:
                self.get_logger().error('Motion execution failed!')
        else:
            self.get_logger().error('Planning failed. Unable to compute a valid trajectory.')

def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
    node = UR3eMotion()

    # Define the target pose
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0

    # Move the robot to the target pose
    node.move_to_pose(target_pose)

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
