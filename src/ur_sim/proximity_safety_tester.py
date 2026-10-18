#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float64
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
import threading

class ProximityTestController(Node):
    def __init__(self):
        super().__init__('proximity_test_controller')
        
        # Test parameters
        self.current_test_phase = 0
        self.phase_duration = 15.0  # seconds per phase
        self.phase_start_time = time.time()
        
        # Human movement parameters
        self.update_rate = 20.0
        self.base_speed = 0.5
        
        # Publishers for human control
        self.model_state_pub = self.create_publisher(
            ModelState,
            '/gazebo/set_model_state',
            10)
            
        self.human_pose_pub = self.create_publisher(
            Pose,
            '/human_pose',
            10)
        
        # Publishers for robot control
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
        
        # Subscribers
        self.speed_scaling_sub = self.create_subscription(
            Float64,
            '/speed_scaling_factor',
            self.speed_scaling_callback,
            10)
            
        self.joint_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.joint_state_callback,
            10)
            
        # Service client for setting model state
        self.set_model_state_client = self.create_client(
            SetModelState,
            '/gazebo/set_model_state')
            
        # Timer for movement updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_positions)
        
        # Robot state
        self.current_joints = None
        self.current_speed_factor = 1.0
        self.robot_moving = False
        
        # Test phases definition
        self.test_phases = [
            {
                'name': 'SAFE ZONE TEST',
                'description': 'Human stays far from robot (>2.0m) - Robot moves at normal speed',
                'center': [3.5, 0.0],
                'radius': 0.5,
                'expected_zone': 'SAFE'
            },
            {
                'name': 'WARNING ZONE TEST', 
                'description': 'Human moves in warning zone (1.5-2.0m) - Robot slows down',
                'center': [1.75, 0.0],
                'radius': 0.25,
                'expected_zone': 'WARNING'
            },
            {
                'name': 'DEAD ZONE TEST',
                'description': 'Human gets very close (<1.5m) - Robot stops completely',
                'center': [1.0, 0.0],
                'radius': 0.4,
                'expected_zone': 'DEAD'
            },
            {
                'name': 'TRANSITION TEST',
                'description': 'Human moves through all zones - Robot speed changes dynamically',
                'center': [0.0, 0.0],
                'radius': 2.2,
                'expected_zone': 'ALL'
            }
        ]
        
        self.get_logger().info('=== PROXIMITY SAFETY TEST CONTROLLER ===')
        self.get_logger().info('This script will test all safety zones:')
        for i, phase in enumerate(self.test_phases):
            self.get_logger().info(f'{i+1}. {phase["name"]}: {phase["description"]}')
        self.get_logger().info('=' * 50)
        
        # Start robot movement
        self.start_robot_movement()
        
    def speed_scaling_callback(self, msg):
        """Receive speed scaling factor from proximity monitor"""
        self.current_speed_factor = msg.data
        
    def joint_state_callback(self, msg):
        """Get current joint positions"""
        if len(msg.actual.positions) >= 6:
            self.current_joints = list(msg.actual.positions[:6])
    
    def start_robot_movement(self):
        """Start continuous robot movement"""
        def robot_mover():
            time.sleep(2.0)  # Wait for system to initialize
            
            # Define target poses for continuous movement
            target_poses = [
                [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],      # Home
                [0.5, -1.2, 0.5, -1.57, 0.0, 0.0],       # Pose 1
                [1.0, -1.0, 1.0, -1.57, 0.0, 0.0],       # Pose 2
                [0.5, -1.2, 0.5, -1.57, 0.0, 0.0],       # Pose 3
                [-0.5, -1.2, 0.5, -1.57, 0.0, 0.0],      # Pose 4
                [-1.0, -1.0, 1.0, -1.57, 0.0, 0.0],      # Pose 5
                [-0.5, -1.2, 0.5, -1.57, 0.0, 0.0],      # Pose 6
            ]
            
            pose_index = 0
            while rclpy.ok():
                try:
                    current_pose = target_poses[pose_index]
                    
                    # Calculate movement duration based on speed scaling
                    base_duration = 4.0
                    scaled_duration = base_duration / max(self.current_speed_factor, 0.1)
                    
                    if self.current_speed_factor < 0.01:
                        # Robot should stop - don't send new trajectory
                        self.get_logger().info('🛑 ROBOT STOPPED due to proximity')
                        time.sleep(1.0)
                        continue
                    
                    self.send_robot_trajectory(current_pose, scaled_duration)
                    
                    speed_status = "NORMAL"
                    if self.current_speed_factor < 0.5:
                        speed_status = "SLOWED"
                    
                    self.get_logger().info(
                        f'🤖 Robot moving to pose {pose_index+1} | '
                        f'Speed: {self.current_speed_factor:.2f} ({speed_status}) | '
                        f'Duration: {scaled_duration:.1f}s'
                    )
                    
                    pose_index = (pose_index + 1) % len(target_poses)
                    time.sleep(scaled_duration + 0.5)
                    
                except Exception as e:
                    self.get_logger().error(f'Robot movement error: {e}')
                    time.sleep(2.0)
        
        # Start robot movement in separate thread
        robot_thread = threading.Thread(target=robot_mover, daemon=True)
        robot_thread.start()
    
    def send_robot_trajectory(self, joint_positions, duration):
        """Send trajectory command to robot"""
        if self.current_joints is None:
            return
            
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
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
        self.joint_trajectory_pub.publish(trajectory)
    
    def calculate_orientation(self, angle):
        """Calculate orientation for human movement"""
        yaw = angle + math.pi/2
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    
    def update_positions(self):
        """Update human position based on current test phase"""
        current_time = time.time()
        
        # Check if we should move to next phase
        if current_time - self.phase_start_time > self.phase_duration:
            self.current_test_phase = (self.current_test_phase + 1) % len(self.test_phases)
            self.phase_start_time = current_time
            
            phase = self.test_phases[self.current_test_phase]
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'🧪 STARTING: {phase["name"]}')
            self.get_logger().info(f'📋 {phase["description"]}')
            self.get_logger().info('=' * 60)
        
        phase = self.test_phases[self.current_test_phase]
        phase_time = current_time - self.phase_start_time
        
        # Calculate human position based on current phase
        angle = self.base_speed * phase_time
        center_x, center_y = phase['center']
        radius = phase['radius']
        
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = 0.0
        
        # Calculate distance from robot (at origin)
        distance = math.sqrt(x*x + y*y)
        
        # Create and publish human pose
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = self.calculate_orientation(angle)
        
        # Publish for proximity monitoring
        self.human_pose_pub.publish(pose)
        
        # Update Gazebo visual
        model_state = ModelState()
        model_state.model_name = 'human_worker'
        model_state.pose = pose
        model_state.reference_frame = 'world'
        self.model_state_pub.publish(model_state)
        
        if self.set_model_state_client.service_is_ready():
            request = SetModelState.Request()
            request.model_state = model_state
            self.set_model_state_client.call_async(request)
        
        # Log status every 2 seconds
        if int(phase_time * 2) % 10 == 0 and phase_time % 0.5 < 1.0/self.update_rate:
            remaining_time = self.phase_duration - phase_time
            
            zone_status = "SAFE"
            if distance <= 1.5:
                zone_status = "🔴 DEAD"
            elif distance <= 2.0:
                zone_status = "🟡 WARNING"
            else:
                zone_status = "🟢 SAFE"
            
            self.get_logger().info(
                f'👤 {phase["name"]} | '
                f'Human: [{x:.2f}, {y:.2f}] | '
                f'Distance: {distance:.2f}m | '
                f'Zone: {zone_status} | '
                f'Robot Speed: {self.current_speed_factor:.2f} | '
                f'Time Left: {remaining_time:.1f}s'
            )

def main():
    rclpy.init()
    
    print("\n" + "="*60)
    print("🤖 PROXIMITY SAFETY TESTING SYSTEM")
    print("="*60)
    print("This script will:")
    print("1. Move the human through different safety zones")
    print("2. Monitor proximity between human and robot")
    print("3. Automatically adjust robot speed based on distance")
    print("4. Test all safety scenarios comprehensively")
    print("="*60)
    print("▶️  Starting in 3 seconds...")
    print("="*60 + "\n")
    
    time.sleep(3)
    
    node = ProximityTestController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Proximity test controller shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()