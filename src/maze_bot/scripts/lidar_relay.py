#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan


class LidarRelay(Node):
    def __init__(self):
        super().__init__('lidar_relay')
        self.declare_parameter('input_topic', '/lidar/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('enabled', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.enabled = bool(self.get_parameter('enabled').value)

        # 雷达同样是高频传感器流（10 Hz、360 线）：使用 BEST_EFFORT，
        # 下游（RViz）来不及消费时直接丢弃旧帧，不会像 RELIABLE 那样
        # 堆积重传队列，最终把整条数据流卡死。RViz 的 LaserScan 显示
        # 也是 Best Effort，双方一致。
        input_qos = QoSProfile(depth=5,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST,
                               durability=DurabilityPolicy.VOLATILE)
        # 只保留最新一帧，下游看到的是“当前扫描”
        output_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST,
                                durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(LaserScan, output_topic,
                                               output_qos)
        self.create_subscription(LaserScan, input_topic, self.on_scan,
                                 input_qos)
        self.get_logger().info(
            f'激光雷达中继就绪：{input_topic} -> {output_topic}'
            f'（frame {self.frame_id}）')

    def on_scan(self, msg):
        if not self.enabled:
            return
        msg.header.frame_id = self.frame_id
        try:
            self.publisher.publish(msg)
        except Exception as exc:  # 单帧发布失败不应终止中继
            self.get_logger().warning(
                f'雷达数据转发失败，已丢弃该帧：{exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()