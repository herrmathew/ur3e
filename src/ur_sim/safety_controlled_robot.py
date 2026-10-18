#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Int32
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
import numpy as np
import time
import math

class SafetyControlledRobot(Node):
    def __init__(self):
        super().__init__('safety_controlled_robot')
        
        # Robot control setup
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Safety parameters
        self.safe_distance = 1.5  # meters
        self.warning_distance = 2.0  # meters
        
        # Robot and human positions
        self.robot_position = None
        self.human_position = None
        self.current_joints = None
        self.robot_moving = False
        self.current_speed_factor = 1.0
        
        # Action client for robot control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Subscribers
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10)
            
        self.human_pose_sub = self.create_subscription(
            Pose,
            '/human_pose',
            self.human_pose_callback,
            10)
            
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        
        # Publishers
        self.speed_scaling_pub = self.create_publisher(
            Float64,
            '/speed_scaling_factor',
            10)
            
        self.proximity_status_pub = self.create_publisher(
            Int32,
            '/proximity_status',
            10)
        
        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10Hz
        
        # Robot movement timer
        self.movement_timer = self.create_timer(8.0, self.plan_next_movement)  # Every 8 seconds
        
        # Predefined poses for testing
        self.test_poses = [
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home
            [0.5, -1.0, 1.0, -1.57, 0.0, 0.0],   # Pose 1
            [1.0, -1.2, 1.2, -1.57, 0.0, 0.0],   # Pose 2  
            [-0.5, -1.0, 1.0, -1.57, 0.0, 0.0],  # Pose 3
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Back to home
        ]
        self.current_pose_index = 0
        
        self.get_logger().info('Safety-controlled robot initialized')
        self.get_logger().info('Waiting for action server and joint states...')
        
        # Wait for services
        self.trajectory_client.wait_for_server()
        
    def model_states_callback(self, msg):
        """Get robot position from Gazebo model states"""
        try:
            # Try different robot names
            robot_names = ['ur', 'spawn_ur3e', 'spawn_ur', 'ur3e', 'robot']
            for name in robot_names:
                try:
                    robot_idx = msg.name.index(name)
                    self.robot_position = np.array([
                        msg.pose[robot_idx].position.x,
                        msg.pose[robot_idx].position.y,
                        msg.pose[robot_idx].position.z
                    ])
                    break
                except ValueError:
                    continue
        except Exception as e:
            self.get_logger().warn(f'Error getting robot position: {str(e)}')
    
    def human_pose_callback(self, msg):
        """Get human position from human_mover"""
        self.human_position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])
    
    def joint_states_callback(self, msg):
        """Get current joint positions"""
        positions = []
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
        if len(positions) == len(self.joint_names):
            self.current_joints = positions
    
    def safety_check(self):
        """Monitor proximity and adjust robot behavior"""
        if self.robot_position is None or self.human_position is None:
            return
            
        # Calculate distance
        distance = np.linalg.norm(self.robot_position - self.human_position)
        
        # Determine safety zone and speed factor
        if distance <= self.safe_distance:
            speed_factor = 0.0
            zone = 2  # Dead zone
            status = "🛑 DEAD ZONE - ROBOT STOPPED"
            
            # STOP THE ROBOT IMMEDIATELY
            if self.robot_moving:
                self.emergency_stop()
                
        elif distance <= self.warning_distance:
            speed_factor = (distance - self.safe_distance) / (self.warning_distance - self.safe_distance)
            zone = 1  # Warning zone  
            status = f"⚠️  WARNING ZONE - ROBOT SLOWED ({speed_factor:.1f}x)"
        else:
            speed_factor = 1.0
            zone = 0  # Safe zone
            status = "✅ SAFE ZONE - NORMAL SPEED"
        
        self.current_speed_factor = speed_factor
        
        # Publish status
        speed_msg = Float64()
        speed_msg.data = speed_factor
        self.speed_scaling_pub.publish(speed_msg)
        
        status_msg = Int32()
        status_msg.data = zone
        self.proximity_status_pub.publish(status_msg)
        
        # Log status
        self.get_logger().info(
            f'{status} | Distance: {distance:.2f}m | '
            f'Human: [{self.human_position[0]:.2f}, {self.human_position[1]:.2f}] | '
            f'Robot Moving: {self.robot_moving}'
        )
    
    def emergency_stop(self):
        """Stop the robot immediately"""
        if self.current_joints is None:
            return
            
        self.get_logger().warn('🚨 EMERGENCY STOP - Human in dead zone!')
        
        # Send immediate stop command (current position)
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_joints
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 second
        
        trajectory.points = [point]
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.trajectory_client.send_goal_async(goal)
        self.robot_moving = False
    
    def move_to_pose(self, joint_positions, duration_base=5.0):
        """Move robot to specified pose with safety speed scaling"""
        if self.current_joints is None:
            self.get_logger().error('No joint states available')
            return False
            
        # Don't move if in dead zone
        if self.current_speed_factor == 0.0:
            self.get_logger().warn('Cannot move - human in dead zone!')
            return False
            
        # Scale duration based on safety
        duration = duration_base / max(self.current_speed_factor, 0.1)  # Slower in warning zone
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Current position
        point1 = JointTrajectoryPoint()
        point1.positions = self.current_joints
        point1.time_from_start.sec = 0
        
        # Target position
        point2 = JointTrajectoryPoint()
        point2.positions = joint_positions
        point2.time_from_start.sec = int(duration)
        point2.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points = [point1, point2]
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f'Moving to pose {self.current_pose_index + 1} (duration: {duration:.1f}s, speed factor: {self.current_speed_factor:.2f})')
        
        self.robot_moving = True
        future = self.trajectory_client.send_goal_async(goal)
        
        # Set timer to mark movement as complete
        self.create_timer(duration + 1.0, lambda: setattr(self, 'robot_moving', False))
        
        return True
    
    def plan_next_movement(self):
        """Plan the next robot movement"""
        if not self.robot_moving:  # Only plan if robot is not currently moving
            target_pose = self.test_poses[self.current_pose_index]
            success = self.move_to_pose(target_pose)
            
            if success:
                self.current_pose_index = (self.current_pose_index + 1) % len(self.test_poses)

def main():
    rclpy.init()
    node = SafetyControlledRobot()
    
    # Wait for initial data
    while rclpy.ok() and node.current_joints is None:
        rclpy.spin_once(node)
        time.sleep(0.1)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Safety-controlled robot shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()