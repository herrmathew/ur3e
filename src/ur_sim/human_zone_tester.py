#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import time

class HumanZoneTester(Node):
    def __init__(self):
        super().__init__('human_zone_tester')
        
        # Movement parameters - designed to test all safety zones
        self.update_rate = 20.0  # Hz
        self.speed = 0.4  # rad/s
        
        # Publishers
        self.human_pose_pub = self.create_publisher(
            Pose,
            '/human_pose',
            10)
            
        self.model_state_pub = self.create_publisher(
            ModelState,
            '/gazebo/set_model_state',
            10)
        
        # Service client for visual human movement
        self.set_model_state_client = self.create_client(
            SetModelState,
            '/gazebo/set_model_state')
        
        # Timer
        self.start_time = time.time()
        self.timer = self.create_timer(1.0/self.update_rate, self.update_position)
        
        self.get_logger().info('🚶 Human Zone Tester initialized')
        self.get_logger().info('Human will move through:')
        self.get_logger().info('  🟢 SAFE ZONE (>2.0m) - Robot normal speed')
        self.get_logger().info('  🟡 WARNING ZONE (1.5-2.0m) - Robot slowed')  
        self.get_logger().info('  🔴 DEAD ZONE (<1.5m) - Robot STOPS')
        
    def calculate_position(self, elapsed_time):
        """Calculate human position to test all safety zones"""
        # Create a path that goes from safe → warning → dead → warning → safe
        
        # Base circular motion with varying radius
        angle = self.speed * elapsed_time
        
        # Vary radius to test different zones
        # This creates a spiral that goes: 2.5m → 1.0m → 2.5m
        radius_variation = 0.75 * math.sin(0.3 * elapsed_time)  # ±0.75m variation
        base_radius = 1.75  # Center around 1.75m
        radius = base_radius + radius_variation
        
        # Position calculation  
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 0.0
        
        # Calculate distance from robot (at origin)
        distance = math.sqrt(x*x + y*y)
        
        return x, y, z, distance
        
    def update_position(self):
        """Update human position"""
        elapsed_time = time.time() - self.start_time
        x, y, z, distance = self.calculate_position(elapsed_time)
        
        # Determine current zone
        if distance <= 1.5:
            zone_emoji = "🔴"
            zone_name = "DEAD ZONE"
        elif distance <= 2.0:
            zone_emoji = "🟡" 
            zone_name = "WARNING"
        else:
            zone_emoji = "🟢"
            zone_name = "SAFE"
        
        # Create pose message
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        
        # Face movement direction
        yaw = math.atan2(y, x) + math.pi/2
        pose.orientation = Quaternion(
            x=0.0, y=0.0, 
            z=math.sin(yaw/2.0), 
            w=math.cos(yaw/2.0)
        )
        
        # Publish human pose for proximity monitoring
        self.human_pose_pub.publish(pose)
        
        # Update visual human in Gazebo
        model_state = ModelState()
        model_state.model_name = 'human_worker'
        model_state.pose = pose
        model_state.reference_frame = 'world'
        
        self.model_state_pub.publish(model_state)
        
        # Use service for more reliable visual updates
        if self.set_model_state_client.service_is_ready():
            request = SetModelState.Request()
            request.model_state = model_state
            self.set_model_state_client.call_async(request)
        
        # Log position every 2 seconds
        if int(elapsed_time * 2) % 20 == 0 and elapsed_time % 0.5 < 1.0/self.update_rate:
            self.get_logger().info(
                f'{zone_emoji} Human in {zone_name} | '
                f'Position: [{x:.2f}, {y:.2f}] | '
                f'Distance: {distance:.2f}m'
            )

def main():
    rclpy.init()
    node = HumanZoneTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Human zone tester shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()