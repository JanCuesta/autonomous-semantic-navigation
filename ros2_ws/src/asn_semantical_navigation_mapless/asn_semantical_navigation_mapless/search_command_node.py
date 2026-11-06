"""Translate a text search request to one validated object category."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from .search_commands import parse_target


class SearchCommandNode(Node):
    def __init__(self):
        super().__init__('search_command_node')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.target_pub = self.create_publisher(
            String, '/semantic/search/target_class', qos)
        self.create_subscription(
            String, '/semantic/search/command', self.on_command, 10)
        self.get_logger().info(
            'Ready for commands: ros2 topic pub --once '
            '/semantic/search/command std_msgs/msg/String '
            "\"{data: 'find a backpack'}\"")

    def on_command(self, msg):
        try:
            category = parse_target(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f'REJECTED request: {exc}')
            return
        self.target_pub.publish(String(data=category))
        self.get_logger().info(
            f'TARGET={category!r}. This only selects a detector class; '
            'it does not verify attributes such as color or start motion.')


def main(args=None):
    rclpy.init(args=args)
    node = SearchCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
