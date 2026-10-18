#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math
import time

class HumanMover(Node):
    def __init__(self):
        super().__init__('human_mover')
        
        # Create publisher for human pose
        self.publisher = self.create_publisher(
            Pose,
            '/human_pose',
            10
        )
        
        # Create timer for periodic updates
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize time tracking
        self.start_time = time.time()
        
        self.get_logger().info('Human mover node started')
        
    def timer_callback(self):
        current_time = time.time() - self.start_time
        
        # Create a circular motion
        radius = 2.0  # 2 meters radius
        angular_speed = 0.5  # radians per second
        
        # Calculate position
        x = radius * math.cos(angular_speed * current_time)
        y = radius * math.sin(angular_speed * current_time)
        z = 0.0
        
        # Create and publish pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        self.publisher.publish(pose)
        self.get_logger().info(f'Publishing human position: [{x:.2f}, {y:.2f}, {z:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    try:
        human_mover = HumanMover()
        rclpy.spin(human_mover)
    except KeyboardInterrupt:
        pass
    finally:
        human_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()