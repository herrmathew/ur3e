#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class HumanMover(Node):
    def __init__(self):
        super().__init__('human_mover')
        
        # Parameters
        self.declare_parameter('radius', 2.0)  # meters
        self.declare_parameter('height', 0.0)  # meters
        self.declare_parameter('center_x', 0.0)  # meters
        self.declare_parameter('center_y', 0.0)  # meters
        self.declare_parameter('speed', 0.5)  # radians per second
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
        self.get_logger().debug(f'Published human position: [{x:.2f}, {y:.2f}, {z:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    node = HumanMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()