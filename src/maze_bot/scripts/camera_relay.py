#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
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

        # 图像体积大（640x480x3 约 900 KB）、发布频率高，必须用 BEST_EFFORT：
        # 一旦下游（RViz）或上游（ros_gz_bridge）来不及消费，直接丢弃旧帧即可，
        # 不会像 RELIABLE 那样堆积重传队列，最终把整条图像流卡死。
        input_qos = QoSProfile(depth=5,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST,
                               durability=DurabilityPolicy.VOLATILE)
        # 只保留最新一帧：下游看到的是“当前画面”，而不是滞后的历史帧
        output_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST,
                                durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Image, output_topic, output_qos)
        self.create_subscription(Image, input_topic, self.on_image, input_qos)
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        self.state_publisher = self.create_publisher(Bool, state_topic, 10)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            f'相机中继就绪：{input_topic} -> {output_topic}（当前'
            f'{"开启" if self.enabled else "关闭"}）')

    def on_image(self, msg):
        if not self.enabled:
            return
        try:
            self.publisher.publish(msg)
        except Exception as exc:  # 单帧发布失败不应终止中继
            self.get_logger().warning(
                f'图像转发失败，已丢弃该帧：{exc}', throttle_duration_sec=5.0)

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