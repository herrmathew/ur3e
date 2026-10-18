import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller')

        # Parameters (tune these)
        self.admittance_gain = 1.0    # K_f (force gain)
        self.damping = 2.0            # D (damping coefficient)
        self.stiffness = 5.0          # K_p (stiffness coefficient)
        self.dt = 0.02                # Time step (50 Hz)
        self.joint_name = 'shoulder_pan_joint'  # Joint to control

        self.force_offset = 0.021      # Estimated force sensor offset (adjust as needed)

        # Velocity limits
        self.max_vel = 0.5

        # State variables
        self.home_position = 0.0
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.latest_force = 0.0
        self.home_set = False

        # Subscriptions
        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.force_callback,
            10)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        # Publisher for velocity command
        self.vel_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

    def force_callback(self, msg):
        # Update latest force with offset compensation
        raw_force = msg.wrench.force.x
        self.latest_force = raw_force - self.force_offset

    def joint_callback(self, msg):
        if self.joint_name in msg.name:
            index = msg.name.index(self.joint_name)
            self.current_position = msg.position[index]

            if not self.home_set:
                self.home_position = self.current_position
                self.home_set = True
                self.get_logger().info(f'Set home position: {self.home_position:.4f}')

    def control_loop(self):
        if not self.home_set:
            return

        # Compute position error relative to home
        position_error = self.current_position - self.home_position

        # Calculate acceleration based on admittance control law
        acceleration = (
            self.admittance_gain * self.latest_force
            - self.damping * self.current_velocity
            - self.stiffness * position_error
        )

        # Integrate acceleration to get velocity
        self.current_velocity += acceleration * self.dt

        # Limit velocity to max allowed
        self.current_velocity = max(min(self.current_velocity, self.max_vel), -self.max_vel)

        # Prepare and publish velocity command message
        msg = Float64MultiArray()
        # Assuming velocity command for 6 joints, only controlling first one here
        msg.data = [self.current_velocity, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_pub.publish(msg)

        # Log detailed debug info
        self.get_logger().info(
            f'HomePos: {self.home_position:.4f}, '
            f'CurrPos: {self.current_position:.4f}, '
            f'PosErr: {position_error:.4f}, '
            f'Force: {self.latest_force:.4f}, '
            f'VelCmd: {self.current_velocity:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

