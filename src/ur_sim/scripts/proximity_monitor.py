#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
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
        
        # Publishers
        self.speed_scaling_pub = self.create_publisher(
            Float64,
            '/speed_scaling_factor',
            10)
        
        # Timer
        self.timer = self.create_timer(1.0/self.get_parameter('check_rate').value, self.check_proximity)
        
        # Store positions
        self.robot_position = None
        self.human_position = None
        
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
            self.get_logger().warn('Could not find robot or human in model states')
    
    def check_proximity(self):
        if self.robot_position is None or self.human_position is None:
            return
            
        distance = np.linalg.norm(self.robot_position - self.human_position)
        
        # Calculate speed scaling factor based on distance
        if distance <= self.safe_distance:
            # Stop the robot
            speed_factor = 0.0
        elif distance <= self.warning_distance:
            # Linear interpolation between 0.0 and 1.0
            speed_factor = (distance - self.safe_distance) / (self.warning_distance - self.safe_distance)
        else:
            # Normal speed
            speed_factor = 1.0
            
        # Publish speed scaling factor
        msg = Float64()
        msg.data = speed_factor
        self.speed_scaling_pub.publish(msg)
        
        # Log status
        if distance <= self.safe_distance:
            self.get_logger().warn(f'Human too close! Stopping robot. Distance: {distance:.2f}m')
        elif distance <= self.warning_distance:
            self.get_logger().info(f'Human in warning zone. Scaling speed to {speed_factor:.2f}. Distance: {distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = ProximityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()