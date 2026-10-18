#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import math

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Create action clients
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd'
        )
        
        # Wait for action servers
        self.trajectory_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        # Joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    async def move_joints(self, positions, duration=2.0):
        """Move joints to specified positions and await result"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.2] * len(positions)
        point.accelerations = [0.1] * len(positions)
        point.time_from_start = Duration(seconds=duration).to_msg()

        trajectory_msg.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        self.get_logger().info(f'Moving to positions: {positions}')
        goal_handle = await self.trajectory_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal was not accepted!')
            return
        result = await goal_handle.get_result_async()
        self.get_logger().info(f'Trajectory result: {result.result}')

    async def gripper_control(self, position, max_effort=5.0):
        """Control the gripper and await result"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        self.get_logger().info(f'Moving gripper to position: {position}')
        goal_handle = await self.gripper_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal was not accepted!')
            return
        result = await goal_handle.get_result_async()
        self.get_logger().info(f'Gripper result: {result.result}')

    async def pick_place_sequence(self):
        """Execute pick and place sequence for each cube using actual world positions"""
        # Cube positions from world file
        cube_positions = [
            (0.6, -0.2, 0.92),  # Red cube
            (0.6, 0.2, 0.92),   # Blue cube
            (0.6, 0.0, 0.92),   # Yellow cube
        ]

        # Place positions (left side of table, same height)
        place_positions = [
            (0.4, -0.2, 0.92),
            (0.4, 0.2, 0.92),
            (0.4, 0.0, 0.92),
        ]

        for pick_pos, place_pos in zip(cube_positions, place_positions):
            # Move to pre-pick position
            await self.move_joints([
                math.atan2(pick_pos[1], pick_pos[0]),  # Pan
                -1.57,  # Lift
                0.0,    # Elbow
                -1.57,  # Wrist 1
                0.0,    # Wrist 2
                0.0     # Wrist 3
            ])
            
            # Open gripper
            await self.gripper_control(0.1)
            
            # Move to pick position
            await self.move_joints([
                math.atan2(pick_pos[1], pick_pos[0]),
                -1.8,
                0.3,
                -1.1,
                0.0,
                0.0
            ])
            
            # Close gripper
            await self.gripper_control(0.7)
            
            # Move to pre-place position
            await self.move_joints([
                math.atan2(place_pos[1], place_pos[0]),
                -1.57,
                0.0,
                -1.57,
                0.0,
                0.0
            ])
            
            # Move to place position
            await self.move_joints([
                math.atan2(place_pos[1], place_pos[0]),
                -1.8,
                0.3,
                -1.1,
                0.0,
                0.0
            ])
            
            # Open gripper to release
            await self.gripper_control(0.1)
            
            # Move back to home position
            await self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

async def main():
    rclpy.init()
    node = PickPlaceDemo()
    
    try:
        await node.pick_place_sequence()
    except Exception as e:
        node.get_logger().error(f'Error during pick and place: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())