import rclpy
from rclpy.node import Node

from robot_interfaces.msg import Encoder


class FakeEncoder(Node):

    def __init__(self):
        super().__init__('fake_encoder_node')

        # Robot name parameter
        self.declare_parameter('robot_name', 'robot1')

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value


        # Topic name
        encoder_topic = f'/{self.robot_name}/encoder'


        # Publisher
        self.encoder_publisher = self.create_publisher(
            Encoder,
            encoder_topic,
            10
        )


        # Fake encoder values
        self.left_ticks = 0
        self.right_ticks = 0


        # Publish every 100ms
        self.timer = self.create_timer(
            0.1,
            self.publish_encoder
        )


        self.get_logger().info(
            f'Fake encoder started for {self.robot_name}'
        )


    def publish_encoder(self):

        msg = Encoder()


        # Simulate forward motion
        # both wheels rotate equally
        self.left_ticks += 10
        self.right_ticks += 10


        msg.left_ticks = self.left_ticks
        msg.right_ticks = self.right_ticks


        self.encoder_publisher.publish(msg)


        self.get_logger().info(
            f'Left: {msg.left_ticks} | Right: {msg.right_ticks}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = FakeEncoder()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()