import rclpy
from rclpy.node import Node

from pid_controller_msgs.srv import SetReference


class ReferenceInputClient(Node):
    def __init__(self):
        super().__init__('reference_input_node')

        # Må matche service-navnet i PID-noden din
        self.client = self.create_client(SetReference, '/pid_controller_node/set_reference')

        self.get_logger().info('Venter på /pid_controller_node/set_reference ...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service ikke tilgjengelig ennå, prøver igjen...')

        self.get_logger().info('Klar! Skriv inn ny referanse (float). Ctrl+C for å avslutte.')

    def send_reference(self, value: float):
        req = SetReference.Request()
        req.request = float(value)

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'Service-kall feilet: {future.exception()}')
            return

        if future.result().success:
            self.get_logger().info(f'Satt referanse til {value}')
        else:
            self.get_logger().warn(f'Referanse {value} avvist (gyldig område: [-pi, pi])')


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceInputClient()

    try:
        while rclpy.ok():
            s = input('Ny referanse: ').strip()
            if not s:
                continue

            try:
                value = float(s)
            except ValueError:
                print("Ugyldig input. Skriv et tall, f.eks 1.0 eller -0.5")
                continue

            node.send_reference(value)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
