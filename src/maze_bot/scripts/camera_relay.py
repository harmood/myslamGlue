#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image')
        self.declare_parameter('enable_topic', '/camera_enable')
        self.declare_parameter('state_topic', '/camera_state')
        self.declare_parameter('enabled', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        state_topic = self.get_parameter('state_topic').value
        self.enabled = bool(self.get_parameter('enabled').value)

        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.create_subscription(Image, input_topic, self.on_image, sensor_qos)
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        self.state_publisher = self.create_publisher(Bool, state_topic, 10)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            f'相机中继就绪：{input_topic} -> {output_topic}（当前'
            f'{"开启" if self.enabled else "关闭"}）')

    def on_image(self, msg):
        if self.enabled:
            self.publisher.publish(msg)

    def on_enable(self, msg):
        if bool(msg.data) != self.enabled:
            self.enabled = bool(msg.data)
            self.publish_state()
            self.get_logger().info('相机已开启' if self.enabled else '相机已关闭')

    def publish_state(self):
        self.state_publisher.publish(Bool(data=self.enabled))


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()