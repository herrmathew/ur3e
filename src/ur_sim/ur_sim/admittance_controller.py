import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import csv
import os
import time

class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')

        # Declare parameters for admittance gain, damping, and stiffness
        self.declare_parameter('admittance_gain', 0.05)
        self.declare_parameter('damping', 10.0)
        self.declare_parameter('stiffness', 50.0)

        self.admittance_gain = self.get_parameter('admittance_gain').get_parameter_value().double_value
        self.damping = self.get_parameter('damping').get_parameter_value().double_value
        self.stiffness = self.get_parameter('stiffness').get_parameter_value().double_value
        self.dt = 0.02  # 50 Hz

        # CONTROLLED JOINT
        self.joint_name = 'shoulder_pan_joint'

        # STATE
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.latest_force = 0.0
        self.home_position = 0.0
        self.home_initialized = False

        # ROS Interfaces
        self.create_subscription(WrenchStamped, '/wrench', self.force_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        # CSV Logging
        self.log_file = os.path.expanduser('~/admittance_log.csv')
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'force', 'pos_error', 'velocity', 'damping', 'stiffness'])

    def force_callback(self, msg):
        self.latest_force = msg.wrench.force.y  # Adjust axis as needed

    def joint_state_callback(self, msg):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]

            if not self.home_initialized:
                self.home_position = self.current_position
                self.home_initialized = True
                self.get_logger().info(f'Set home position: {self.home_position:.4f}')

    def control_loop(self):
        if not self.home_initialized:
            return

        # Deadband to avoid noise
        force_threshold = 0.1
        force_input = self.latest_force if abs(self.latest_force) > force_threshold else 0.0

        pos_error = self.current_position - self.home_position

        # Refresh parameters dynamically
        self.admittance_gain = self.get_parameter('admittance_gain').get_parameter_value().double_value
        self.damping = self.get_parameter('damping').get_parameter_value().double_value
        self.stiffness = self.get_parameter('stiffness').get_parameter_value().double_value

        # Admittance equation: a = F - Dv - Kx
        acceleration = (
            self.admittance_gain * force_input
            - self.damping * self.current_velocity
            - self.stiffness * pos_error
        )

        self.current_velocity += acceleration * self.dt

        # Clamp velocity
        max_vel = 0.5
        self.current_velocity = max(min(self.current_velocity, max_vel), -max_vel)

        # Publish velocity command
        msg = Float64MultiArray()
        msg.data = [self.current_velocity, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_pub.publish(msg)

        # Log to console
        self.get_logger().info(
            f'Force: {self.latest_force:.4f}, PosErr: {pos_error:.4f}, Vel: {self.current_velocity:.4f}, '
            f'Damping: {self.damping:.2f}, Stiffness: {self.stiffness:.2f}, Gain: {self.admittance_gain:.4f}'
        )

        # Log to CSV
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.time(),
                self.latest_force,
                pos_error,
                self.current_velocity,
                self.damping,
                self.stiffness
            ])

def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

