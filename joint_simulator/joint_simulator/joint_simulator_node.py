import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
from pid_controller_msgs.srv import SetReference



class jointSimulator:
    def __init__(self, angle, angular_velocity, voltage, K, T, noise):
        self.angle = angle
        self.angular_velocity = angular_velocity
        self.voltage = voltage
        self.noise = noise

        self.K = K
        self.T = T

    def update(self, dt):
        angular_acceleration = (self.K * self.voltage - self.angular_velocity) / self.T
        self.angular_velocity += dt * angular_acceleration
        self.angle += dt * self.angular_velocity + self.noise
        return self.angle


class JointSimulatorNode(Node):
    def __init__(self):
        super().__init__('joint_simulator_node')

        # 🔹 Declare parameters
        self.declare_parameter('noise', 0.0)
        self.declare_parameter('K', 230.0)
        self.declare_parameter('T', 0.15)

        # 🔹 Get parameters
        noise = self.get_parameter('noise').value
        K = self.get_parameter('K').value
        T = self.get_parameter('T').value

        # Simulator-instans
        self.simulator = jointSimulator(
            angle=0.0,
            angular_velocity=0.0,
            voltage=0.0,
            K=K,
            T=T,
            noise=noise
        )
        # self.srv = self.create_service(SetReference, 'SetReference', self.parameter_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Publisher
        self.publish_angle = self.create_publisher(
            Float64,
            'angle',
            10
        )

        # Subscriber
        self.input_voltage = self.create_subscription(
            Float64,
            'voltage',
            self.voltage_listener,
            10
        )

        # Tidssteg
        self.dt = 0.01

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'Joint simulator started (K={K}, T={T}, noise={noise})'
        )

    def voltage_listener(self, msg):
        self.simulator.voltage = msg.data

    def timer_callback(self):
        self.simulator.update(self.dt)

        msg = Float64()
        msg.data = self.simulator.angle
        self.publish_angle.publish(msg)
    
    def parameter_callback(self, params):
        """Callback to handle parameter updates."""
        for param in params:
            if param.name == 'noise':
                if (param.value >= 0.0):
                    self.simulator.noise = param.value
                    self.get_logger().info(f' Noise was set to: {self.simulator.noise}')
            if param.name == 'K':
                if (param.value >= 0.0):
                    self.simulator.K = param.value
                    self.get_logger().info(f' K was set to: {self.simulator.K}')
            if param.name == 'T':
                if (param.value >= 0.0):
                    self.simulator.T = param.value
                    self.get_logger().info(f' T was set to: {self.simulator.T}')
        return SetParametersResult(successful=True)
    
def main(args=None):
    rclpy.init(args=args)
    node = JointSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
