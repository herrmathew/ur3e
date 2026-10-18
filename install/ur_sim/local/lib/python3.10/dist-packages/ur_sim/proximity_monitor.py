#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
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
        self.declare_parameter('robot_name', 'spawn_ur3e')  # Robot name in Gazebo
        self.declare_parameter('debug', True)  # Enable debug output
        
        self.safe_distance = self.get_parameter('safe_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.robot_name = self.get_parameter('robot_name').value
        self.debug = self.get_parameter('debug').value
        
        # Initialize positions
        self.robot_position = None
        self.human_position = None
        self.last_human_update = None
        self.last_robot_update = None
        self._last_model_states_time = self.get_clock().now()
        
        # Subscribers
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10)
            
        # Check if we're receiving model states
        self.create_timer(1.0, self.check_model_states_connection)
            
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
            
        self.get_logger().info('Starting proximity monitor. Waiting for robot model...')
        
        # Timer
        self.timer = self.create_timer(0.1, self.check_proximity)  # 10Hz fixed rate
        
        # Store positions
        self.robot_position = None
        self.human_position = None
        self.last_human_update = self.get_clock().now()
        self.last_robot_update = self.get_clock().now()
        
        # Status tracking
        self.last_published_zone = -1  # Initialize to invalid zone
        
        self.get_logger().info('Proximity monitor initialized')
        
    def model_states_callback(self, msg):
        try:
            self._last_model_states_time = self.get_clock().now()
            # Print all available model names when they change
            if not hasattr(self, '_last_models') or self._last_models != msg.name:
                self.get_logger().info('===== Available models in Gazebo =====')
                for i, name in enumerate(msg.name):
                    self.get_logger().info(f'{i}: {name}')
                self.get_logger().info('====================================')
                self._last_models = msg.name

            # Print model list when it changes
            if not hasattr(self, '_last_models') or self._last_models != msg.name:
                self.get_logger().info('===== Models in Gazebo =====')
                for i, name in enumerate(msg.name):
                    self.get_logger().info(f'{i}: {name}')
                self.get_logger().info('===========================')
                self._last_models = msg.name.copy()

            # Try to find robot with different possible names
            robot_idx = None
            
            # First try exact match for 'ur' which is the known name
            try:
                robot_idx = msg.name.index('ur')
                if not hasattr(self, '_found_robot'):
                    self.get_logger().info(f'Found robot with name: ur at index {robot_idx}')
                    self._found_robot = True
            except ValueError:
                # If 'ur' not found, try alternative names
                robot_names = ['spawn_ur3e', 'ur3e', 'ur3e_robot', 'robot', 'spawn_ur']
                for name in robot_names:
                    try:
                        robot_idx = msg.name.index(name)
                        if not hasattr(self, '_found_robot'):
                            self.get_logger().info(f'Found robot with alternate name: {name} at index {robot_idx}')
                            self._found_robot = True
                        break
                    except ValueError:
                        continue

            if robot_idx is not None:
                # Get the robot position
                robot_pos = np.array([
                    msg.pose[robot_idx].position.x,
                    msg.pose[robot_idx].position.y,
                    msg.pose[robot_idx].position.z
                ])

                # Update position and timestamp
                self.robot_position = robot_pos
                self.last_robot_update = self.get_clock().now()

                # Always log the position for debugging
                self.get_logger().info(
                    f'Robot found - Name: {msg.name[robot_idx]} | '
                    f'Position: [{robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}]'
                )
            else:
                self.get_logger().warn(
                    f'Robot not found. Searched for: {robot_names}'
                )
            
            if robot_idx is None:
                self.get_logger().warn('Robot not found in model list')
                return
            
        except Exception as e:
            self.get_logger().error(f'Error getting robot position: {str(e)}')
            
    def human_pose_callback(self, msg):
        self.human_position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])
        self.last_human_update = self.get_clock().now()
        if self.debug:
            self.get_logger().info(f'Human position updated: [{msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}]')
        self.last_human_update = self.get_clock().now()
        self.get_logger().info(f'Received human position: [{msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}]')
    
    def check_proximity(self):
        current_time = self.get_clock().now()
        
        # Always publish status even if no updates (helps with monitoring)
        status_msg = Int32()
        
        if self.robot_position is None:
            self.get_logger().warn('No robot position available')
            status_msg.data = -1  # Error state
            self.proximity_status_pub.publish(status_msg)
            return
            
        if self.human_position is None:
            self.get_logger().warn('No human position available')
            status_msg.data = -2  # Error state
            self.proximity_status_pub.publish(status_msg)
            return
            
        # Check if updates are too old (2 seconds threshold)
        if (current_time - self.last_human_update).nanoseconds / 1e9 > 2.0:
            self.get_logger().warn('Human position data is stale')
            status_msg.data = -3  # Stale data
            self.proximity_status_pub.publish(status_msg)
            return
            
        distance = np.linalg.norm(self.robot_position - self.human_position)
        
        # Calculate speed scaling factor based on distance
        if distance <= self.safe_distance:
            speed_factor = 0.0
            zone = 2  # Stop zone
            status = "STOP"
        elif distance <= self.warning_distance:
            speed_factor = (distance - self.safe_distance) / (self.warning_distance - self.safe_distance)
            zone = 1  # Warning zone
            status = "WARNING"
        else:
            speed_factor = 1.0
            zone = 0  # Safe zone
            status = "SAFE"
            
        # Only publish and log if zone changed or every 1 second
        if zone != self.last_published_zone or (current_time - self.last_human_update).nanoseconds / 1e9 >= 1.0:
            # Publish speed scaling factor
            speed_msg = Float64()
            speed_msg.data = speed_factor
            self.speed_scaling_pub.publish(speed_msg)
            
            # Publish proximity status
            status_msg.data = zone
            self.proximity_status_pub.publish(status_msg)
            
            # Log status with detailed information
            self.get_logger().info(
                f'Status: {status} | ' +
                f'Distance: {distance:.2f}m | ' +
                f'Zone: {zone} | ' +
                f'Speed Factor: {speed_factor:.2f} | ' +
                f'Human Pos: [{self.human_position[0]:.2f}, {self.human_position[1]:.2f}, {self.human_position[2]:.2f}]'
            )
            
            self.last_published_zone = zone

    def check_model_states_connection(self):
        current_time = self.get_clock().now()
        if (current_time - self._last_model_states_time).nanoseconds / 1e9 > 2.0:
            self.get_logger().warn('No model states messages received for 2 seconds. Check if Gazebo is running and publishing to /gazebo/model_states')

def main(args=None):
    rclpy.init(args=args)
    node = ProximityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()