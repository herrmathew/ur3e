#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import time

class HumanActorController(Node):
    def __init__(self):
        super().__init__('human_actor_controller')
        
        # Parameters
        self.declare_parameter('radius', 2.5)  # meters
        self.declare_parameter('height', 0.0)  # meters
        self.declare_parameter('center_x', 0.0)  # meters
        self.declare_parameter('center_y', 0.0)  # meters
        self.declare_parameter('speed', 0.3)  # radians per second
        self.declare_parameter('update_rate', 20.0)  # Hz
        
        self.radius = self.get_parameter('radius').value
        self.height = self.get_parameter('height').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.speed = self.get_parameter('speed').value
        self.update_rate = self.get_parameter('update_rate').value
        
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
        self.get_logger().info(f'  Radius: {self.radius}m')
        self.get_logger().info(f'  Height: {self.height}m')
        self.get_logger().info(f'  Center: ({self.center_x}, {self.center_y})')
        self.get_logger().info(f'  Speed: {self.speed} rad/s')
        self.get_logger().info(f'  Update Rate: {self.update_rate} Hz')
        
        # Wait for service
        self.get_logger().info('Waiting for /gazebo/set_model_state service...')
        
    def calculate_orientation(self, current_angle):
        """Calculate orientation quaternion for the human to face movement direction"""
        # The human should face the direction of movement
        yaw = current_angle + math.pi/2  # Offset so human faces forward in movement
        
        # Convert yaw to quaternion
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
        
        # Also try using the service (more reliable)
        if self.set_model_state_client.service_is_ready():
            request = SetModelState.Request()
            request.model_state = model_state
            # Fire and forget - don't wait for response to avoid blocking
            self.set_model_state_client.call_async(request)
        
        # Log position occasionally
        if int(current_time * 2) % 10 == 0 and current_time % 0.5 < 1.0/self.update_rate:
            self.get_logger().info(f'Human position: [{x:.2f}, {y:.2f}, {z:.2f}]')

def main():
    rclpy.init()
    node = HumanActorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Human actor controller shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()