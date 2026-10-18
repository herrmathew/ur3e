#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Create the action client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        
        # Subscribe to joint states
        self.current_joints = None
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
    def joint_states_callback(self, msg):
        # Get the positions of the arm joints
        positions = []
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
        if len(positions) == len(self.joint_names):
            self.current_joints = positions

    def move_to_pose(self, joint_positions, duration=5.0):
        if self.current_joints is None:
            self.get_logger().error('No joint states received yet!')
            return False
            
        # Create the trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Add current position as first point
        point1 = JointTrajectoryPoint()
        point1.positions = self.current_joints
        point1.time_from_start.sec = 0
        
        # Add target position as second point
        point2 = JointTrajectoryPoint()
        point2.positions = joint_positions
        point2.time_from_start.sec = int(duration)
        point2.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points = [point1, point2]
        
        # Create and send the action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info('Sending goal...')
        self.trajectory_client.send_goal_async(goal)
        self.get_logger().info('Goal sent!')
        return True

def main():
    """Entry point for ROS2 executable."""
    rclpy.init()
    node = RobotMover()
    
    # Wait for joint states
    while rclpy.ok() and node.current_joints is None:
        rclpy.spin_once(node)
        time.sleep(0.1)
    
    node.get_logger().info('Robot mover ready! Use move_to_pose() method to control the robot.')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def demo_main():
    """Demo function that shows example movements."""
    rclpy.init()
    node = RobotMover()
    
    # Wait for joint states
    while rclpy.ok() and node.current_joints is None:
        rclpy.spin_once(node)
        time.sleep(0.1)
    
    # Example poses
    home_pose = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # Home position
    example_poses = [
        [0.0, -1.0, 1.0, -1.57, 0.0, 0.0],  # Pose 1
        [1.57, -1.0, 1.0, -1.57, 0.0, 0.0],  # Pose 2
        [-1.57, -1.0, 1.0, -1.57, 0.0, 0.0],  # Pose 3
    ]
    
    # Move to home position first
    node.get_logger().info('Moving to home position...')
    node.move_to_pose(home_pose)
    time.sleep(6.0)  # Wait for motion to complete
    
    # Move through example poses
    for i, pose in enumerate(example_poses):
        node.get_logger().info(f'Moving to pose {i+1}...')
        node.move_to_pose(pose)
        time.sleep(6.0)  # Wait for motion to complete
    
    # Return to home position
    node.get_logger().info('Returning to home position...')
    node.move_to_pose(home_pose)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    demo_main()