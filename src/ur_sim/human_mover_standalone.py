#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time
import sys
import argparse

class HumanMover(Node):
    def __init__(self):
        super().__init__('human_mover')
        
        # Parameters
        self.declare_parameter('radius', 2.0)  # meters
        self.declare_parameter('height', 1.0)  # meters (human height)
        self.declare_parameter('center_x', 2.0)  # meters (offset from robot)
        self.declare_parameter('center_y', 2.0)  # meters
        self.declare_parameter('speed', 0.3)  # radians per second (slower movement)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        self.radius = self.get_parameter('radius').value
        self.height = self.get_parameter('height').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.speed = self.get_parameter('speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.pose_publisher = self.create_publisher(
            Pose,
            '/human_pose',
            10)
            
        # Timer for movement updates
        self.start_time = time.time()
        self.timer = self.create_timer(1.0/self.publish_rate, self.move_human)
        
        self.get_logger().info('Human mover initialized with parameters:')
        self.get_logger().info(f'  Radius: {self.radius}m')
        self.get_logger().info(f'  Height: {self.height}m')
        self.get_logger().info(f'  Center: ({self.center_x}, {self.center_y})')
        self.get_logger().info(f'  Speed: {self.speed} rad/s')
        self.get_logger().info(f'  Rate: {self.publish_rate} Hz')
        self.get_logger().info('Publishing human poses to /human_pose topic')
        
    def move_human(self):
        current_time = time.time() - self.start_time
        angle = self.speed * current_time
        
        # Calculate position in circular path
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)
        z = self.height
        
        # Create and publish pose message
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        
        # Set orientation to identity quaternion
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        self.pose_publisher.publish(pose)
        
        # Log position every few seconds
        if int(current_time) % 5 == 0 and current_time % 1 < 0.1:
            self.get_logger().info(f'Human position: [{x:.2f}, {y:.2f}, {z:.2f}]')

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Human Mover Node')
    parser.add_argument('--radius', type=float, default=2.0, help='Movement radius (m)')
    parser.add_argument('--height', type=float, default=1.0, help='Human height (m)')
    parser.add_argument('--center_x', type=float, default=2.0, help='Center X position (m)')
    parser.add_argument('--center_y', type=float, default=2.0, help='Center Y position (m)')
    parser.add_argument('--speed', type=float, default=0.3, help='Movement speed (rad/s)')
    parser.add_argument('--rate', type=float, default=10.0, help='Publishing rate (Hz)')
    
    # Filter out ROS2 arguments
    args, unknown = parser.parse_known_args()
    
    rclpy.init(args=sys.argv)
    
    # Create node with parameters
    node = HumanMover()
    
    # Override parameters from command line
    node.set_parameters([
        rclpy.parameter.Parameter('radius', rclpy.Parameter.Type.DOUBLE, args.radius),
        rclpy.parameter.Parameter('height', rclpy.Parameter.Type.DOUBLE, args.height),
        rclpy.parameter.Parameter('center_x', rclpy.Parameter.Type.DOUBLE, args.center_x),
        rclpy.parameter.Parameter('center_y', rclpy.Parameter.Type.DOUBLE, args.center_y),
        rclpy.parameter.Parameter('speed', rclpy.Parameter.Type.DOUBLE, args.speed),
        rclpy.parameter.Parameter('publish_rate', rclpy.Parameter.Type.DOUBLE, args.rate),
    ])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Human mover shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()