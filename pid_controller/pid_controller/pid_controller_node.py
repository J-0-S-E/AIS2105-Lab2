import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult

from pid_controller_msgs.srv import SetReference


class pidController:
    def __init__(self, p, i, d, reference, voltage):
        self.p = p
        self.i = i
        self.d = d
        self.last_error = 0
        self.integral = 0
        self.reference = reference
        self.voltage = voltage
        self.last_time = time.time()

    def update(self, measurement):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return 0

        error = self.reference - measurement

        P_term = self.p * error

        self.integral += error * dt
        I_term = self.i * self.integral

        derivative = (error - self.last_error) / dt
        D_term = self.d * derivative

        self.voltage = P_term + I_term + D_term

        self.last_error = error
        self.last_time = current_time

        return self.voltage


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Parametere
        self.declare_parameter('p', 0.1)
        self.declare_parameter('i', 0.01)
        self.declare_parameter('d', 0.001)
        self.declare_parameter('r', 1.0)

        self.p = float(self.get_parameter('p').value)
        self.i = float(self.get_parameter('i').value)
        self.d = float(self.get_parameter('d').value)
        self.reference = float(self.get_parameter('r').value)

        self.pid = pidController(
            p=self.p,
            i=self.i,
            d=self.d,
            reference=self.reference,
            voltage=0.0
        )

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.publish_voltage = self.create_publisher(Float64, 'voltage', 10)

        self.measured_angle = self.create_subscription(
            Float64,
            'angle',
            self.measurement_listener,
            10
        )

        # Service-server: set_reference
        self.set_reference_srv = self.create_service(
            SetReference,
             '/pid_controller_node/set_reference',
            self.set_reference_callback
        )
        self.get_logger().info('Service /set_reference klar (gyldig område: [-pi, pi]).')

    def set_reference_callback(self, request: SetReference.Request, response: SetReference.Response):
        ref = float(request.request)

        if -math.pi <= ref <= math.pi:
            self.reference = ref
            self.pid.reference = ref
            response.success = True
            self.get_logger().info(f'Satt referanse via service til {ref:.6f}')
        else:
            response.success = False
            self.get_logger().warn(f'Ugyldig referanse {ref:.6f}. Må være i [-pi, pi].')

        return response

    # Parameter-callback (som før)
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p':
                if param.value >= 0.0:
                    self.p = float(param.value)
                    self.pid.p = self.p
                    self.get_logger().info(f'p was set: {self.p}')

            elif param.name == 'i':
                if param.value >= 0.0:
                    self.i = float(param.value)
                    self.pid.i = self.i
                    self.get_logger().info(f'i was set: {self.i}')

            elif param.name == 'd':
                if param.value >= 0.0:
                    self.d = float(param.value)
                    self.pid.d = self.d
                    self.get_logger().info(f'd was set: {self.d}')

            elif param.name == 'r':
                if param.value >= 0.0:
                    # Litt ryddigere: bruk param.value direkte (i stedet for get_parameter('r') igjen)
                    self.reference = float(param.value)
                    self.pid.reference = self.reference
                    self.get_logger().info(f'reference was set: {self.reference}')

        return SetParametersResult(successful=True)

    def measurement_listener(self, msg):
        out_msg = Float64()
        out_msg.data = self.pid.update(msg.data)
        self.publish_voltage.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
