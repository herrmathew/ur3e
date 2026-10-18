#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64, Int32
from gazebo_msgs.msg import ModelStates
import numpy as np

class ProximityMonitor(Node):
    def __init__(self):
        super().__init__('proximity_monitor')
        
        # Parameters
        self.declare_parameter('safe_distance', 1.5)  # meters
        self.declare_parameter('warning_distance', 2.0)  # meters
        self.declare_parameter('check_rate', 10.0)  # Hz
        
        self.safe_distance = self.get_parameter('safe_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        
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
        
        # Publishers
        self.speed_scaling_pub = self.create_publisher(
            Float64,
            '/speed_scaling_factor',
            10)
            
        self.proximity_status_pub = self.create_publisher(
            Int32,
            '/proximity_status',
            10)
        
        # Timer
        self.timer = self.create_timer(1.0/self.get_parameter('check_rate').value, self.check_proximity)
        
        # Store positions
        self.robot_position = None
        self.human_position = None
        self.last_human_update = self.get_clock().now()
        self.last_robot_update = self.get_clock().now()
        
        self.get_logger().info('Proximity Monitor initialized - watching for robot and human positions')
        
    def model_states_callback(self, msg):
        try:
            robot_idx = msg.name.index('ur')
            human_idx = msg.name.index('human_worker')
            
            self.robot_position = np.array([
                msg.pose[robot_idx].position.x,
                msg.pose[robot_idx].position.y,
                msg.pose[robot_idx].position.z
            ])
            
            self.human_position = np.array([
                msg.pose[human_idx].position.x,
                msg.pose[human_idx].position.y,
                msg.pose[human_idx].position.z
            ])
        except ValueError:
            # Try to find robot with different possible names
            robot_names = ['ur', 'spawn_ur3e', 'ur3e', 'robot']
            robot_idx = None
            for name in robot_names:
                try:
                    robot_idx = msg.name.index(name)
                    self.robot_position = np.array([
                        msg.pose[robot_idx].position.x,
                        msg.pose[robot_idx].position.y,
                        msg.pose[robot_idx].position.z
                    ])
                    self.last_robot_update = self.get_clock().now()
                    break
                except ValueError:
                    continue
            
            if robot_idx is None:
                self.get_logger().warn('Could not find robot in model states')
    
    def human_pose_callback(self, msg):
        """Callback for human pose from /human_pose topic"""
        self.human_position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])
        self.last_human_update = self.get_clock().now()
        self.get_logger().debug(f'Human pose updated: [{msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}]')
    
    def check_proximity(self):
        current_time = self.get_clock().now()
        
        # Check if we have both positions
        if self.robot_position is None:
            self.get_logger().warn('No robot position available')
            return
            
        if self.human_position is None:
            self.get_logger().warn('No human position available')
            return
            
        # Check if data is too old
        if (current_time - self.last_human_update).nanoseconds / 1e9 > 2.0:
            self.get_logger().warn('Human position data is stale')
            return
            
        distance = np.linalg.norm(self.robot_position - self.human_position)
        
        # Calculate speed scaling factor based on distance
        if distance <= self.safe_distance:
            speed_factor = 0.0
            zone = 2  # Dead zone
            status = "DEAD ZONE - ROBOT STOPPED"
        elif distance <= self.warning_distance:
            speed_factor = (distance - self.safe_distance) / (self.warning_distance - self.safe_distance)
            zone = 1  # Warning zone
            status = "WARNING ZONE - ROBOT SLOWED"
        else:
            speed_factor = 1.0
            zone = 0  # Safe zone
            status = "SAFE ZONE - NORMAL SPEED"
            
        # Publish speed scaling factor
        speed_msg = Float64()
        speed_msg.data = speed_factor
        self.speed_scaling_pub.publish(speed_msg)
        
        # Publish proximity status
        status_msg = Int32()
        status_msg.data = zone
        self.proximity_status_pub.publish(status_msg)
        
        # Log detailed status
        self.get_logger().info(
            f'{status} | Distance: {distance:.2f}m | Speed Factor: {speed_factor:.2f} | '
            f'Robot: [{self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}] | '
            f'Human: [{self.human_position[0]:.2f}, {self.human_position[1]:.2f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ProximityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()