#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import time
import argparse
import sys

class HumanActorController(Node):
    def __init__(self, radius=1.2, speed=0.4, center_x=0.8, center_y=0.8):
        super().__init__('human_actor_controller')
        
        # Use provided parameters
        self.radius = radius
        self.height = 0.0
        self.center_x = center_x
        self.center_y = center_y
        self.speed = speed
        self.update_rate = 20.0
        
        # Publishers
        self.model_state_pub = self.create_publisher(
            ModelState,
            '/gazebo/set_model_state',
            10)
            
        self.human_pose_pub = self.create_publisher(
            Pose,
            '/human_pose',
            10)
            
        # Service client for setting model state
        self.set_model_state_client = self.create_client(
            SetModelState,
            '/gazebo/set_model_state')
            
        # Timer for movement updates
        self.start_time = time.time()
        self.timer = self.create_timer(1.0/self.update_rate, self.update_human_position)
        
        self.get_logger().info('Human actor controller initialized:')
        self.get_logger().info(f'  Radius: {self.radius}m (brings human close to robot)')
        self.get_logger().info(f'  Center: ({self.center_x}, {self.center_y}) (offset from robot at origin)')
        self.get_logger().info(f'  Speed: {self.speed} rad/s')
        self.get_logger().info('Human will move through WARNING ZONE (1.5-2.0m) and DEAD ZONE (<1.5m)')
        
    def calculate_orientation(self, current_angle):
        """Calculate orientation quaternion for the human to face movement direction"""
        yaw = current_angle + math.pi/2
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
    def update_human_position(self):
        current_time = time.time() - self.start_time
        angle = self.speed * current_time
        
        # Calculate position in circular path
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)
        z = self.height
        
        # Calculate distance from robot (at origin 0,0,0)
        distance_from_robot = math.sqrt(x*x + y*y)
        
        # Create pose
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = self.calculate_orientation(angle)
        
        # Publish human pose for proximity monitoring
        self.human_pose_pub.publish(pose)
        
        # Update Gazebo model state
        model_state = ModelState()
        model_state.model_name = 'human_worker'
        model_state.pose = pose
        model_state.reference_frame = 'world'
        
        # Publish model state
        self.model_state_pub.publish(model_state)
        
        # Use service for more reliable updates
        if self.set_model_state_client.service_is_ready():
            request = SetModelState.Request()
            request.model_state = model_state
            self.set_model_state_client.call_async(request)
        
        # Log position and zone information
        if int(current_time * 2) % 10 == 0 and current_time % 0.5 < 1.0/self.update_rate:
            zone = "SAFE"
            if distance_from_robot <= 1.5:
                zone = "DEAD ZONE"
            elif distance_from_robot <= 2.0:
                zone = "WARNING"
            
            self.get_logger().info(
                f'Human: [{x:.2f}, {y:.2f}] | Distance: {distance_from_robot:.2f}m | Zone: {zone}'
            )

def main():
    parser = argparse.ArgumentParser(description='Human Actor Controller for Testing Safety Zones')
    parser.add_argument('--test-mode', choices=['warning', 'dead', 'all'], default='all',
                       help='Test specific safety zone: warning, dead, or all zones')
    parser.add_argument('--radius', type=float, default=1.2, help='Movement radius (m)')
    parser.add_argument('--speed', type=float, default=0.4, help='Movement speed (rad/s)')
    parser.add_argument('--center_x', type=float, default=0.8, help='Center X position (m)')
    parser.add_argument('--center_y', type=float, default=0.8, help='Center Y position (m)')
    
    args, unknown = parser.parse_known_args()
    
    # Adjust parameters based on test mode
    if args.test_mode == 'warning':
        # Keep human mostly in warning zone (1.5-2.0m)
        radius = 0.25
        center_x, center_y = 1.75, 0.0
        print("Testing WARNING ZONE: Human will stay around 1.5-2.0m from robot")
    elif args.test_mode == 'dead':
        # Keep human mostly in dead zone (<1.5m)
        radius = 0.5
        center_x, center_y = 1.0, 0.0
        print("Testing DEAD ZONE: Human will stay within 1.5m of robot")
    else:
        # Test all zones
        radius = args.radius
        center_x, center_y = args.center_x, args.center_y
        print("Testing ALL ZONES: Human will move through safe, warning, and dead zones")
    
    rclpy.init(args=sys.argv)
    node = HumanActorController(
        radius=radius,
        speed=args.speed,
        center_x=center_x,
        center_y=center_y
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Human actor controller shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()