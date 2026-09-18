#!/usr/bin/env python3
"""相机图像中继节点。

链路：Gazebo 相机 -> /camera/image_raw（ros_gz_bridge）-> 本节点 -> /camera/image -> RViz。

存在的意义有两点：
    1. QoS 适配：Gazebo 桥上来的图像话题是 RELIABLE，而 RViz 的 Image 显示默认
       Best Effort。本节点用 BEST_EFFORT 转发，下游来不及消费时直接丢帧，
       不会堆积重传队列把整条图像流堵死（曾出现“先能看、过一会儿就收不到”）。
    2. 开关控制：订阅 /camera_enable（std_msgs/Bool），可临时关闭图像转发，
       方便在不看画面时省下带宽；状态通过 /camera_state 反馈给界面。
"""
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
        # 话题与开关均可通过参数覆盖，默认值对应 spawn_maze.launch.py 的配置
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
        # 开关用默认 QoS 即可（低频布尔量，需要可靠送达）
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        self.state_publisher = self.create_publisher(Bool, state_topic, 10)
        # 周期上报开关状态，让后启动的订阅者（如 GUI）也能拿到当前值
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info(
            f'相机中继就绪：{input_topic} -> {output_topic}（当前'
            f'{"开启" if self.enabled else "关闭"}）')

    def on_image(self, msg):
        """收到一帧图像：开启状态才转发。"""
        if not self.enabled:
            return
        try:
            self.publisher.publish(msg)
        except Exception as exc:  # 单帧发布失败不应终止中继
            # 限频 5 秒，避免持续失败时刷屏
            self.get_logger().warning(
                f'图像转发失败，已丢弃该帧：{exc}', throttle_duration_sec=5.0)

    def on_enable(self, msg):
        """处理 /camera_enable 开关指令，仅在状态真正变化时动作。"""
        if bool(msg.data) != self.enabled:
            self.enabled = bool(msg.data)
            self.publish_state()
            self.get_logger().info('相机已开启' if self.enabled else '相机已关闭')

    def publish_state(self):
        """把当前开关状态发布到 /camera_state。"""
        self.state_publisher.publish(Bool(data=self.enabled))


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 无论正常退出还是异常，都释放节点资源
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()