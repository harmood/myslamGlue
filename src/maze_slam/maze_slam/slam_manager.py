#!/usr/bin/env python3
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger

from maze_slam.resource_paths import default_map_dir

try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    point_cloud2 = None


class SlamManager(Node):
    def __init__(self):
        super().__init__('slam_manager')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('slam_image_topic', '/slam/image')
        self.declare_parameter('slam_depth_topic', '/slam/depth_image')
        self.declare_parameter('camera_info_topic', '/slam/camera_info')
        self.declare_parameter('depth_info_topic', '/slam/depth_camera_info')
        self.declare_parameter('enable_topic', '/slam_enable')
        self.declare_parameter('state_topic', '/slam_state')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('pause_service', '/rtabmap/pause')
        self.declare_parameter('resume_service', '/rtabmap/resume')
        self.declare_parameter('map_dir', '')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('horizontal_fov', 1.047)
        self.declare_parameter('enabled', False)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        slam_image_topic = self.get_parameter('slam_image_topic').value
        slam_depth_topic = self.get_parameter('slam_depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        state_topic = self.get_parameter('state_topic').value
        map_topic = self.get_parameter('map_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        self.hfov = float(self.get_parameter('horizontal_fov').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.map_dir = self.resolve_map_dir(self.get_parameter('map_dir').value)
        self.latest_map = None
        self.latest_cloud = None

        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        slam_qos = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.image_publisher = self.create_publisher(Image, slam_image_topic,
                                                     slam_qos)
        self.depth_publisher = self.create_publisher(Image, slam_depth_topic,
                                                     slam_qos)
        self.info_publisher = self.create_publisher(CameraInfo,
                                                    camera_info_topic, 5)
        self.depth_info_publisher = self.create_publisher(CameraInfo,
                                                          depth_info_topic, 5)
        self.state_publisher = self.create_publisher(Bool, state_topic, 10)
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.create_subscription(Image, depth_topic, self.on_depth, sensor_qos)
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud,
                                 map_qos)
        self.create_service(Trigger, '~/save_map', self.on_save_map)
        self.pause_client = self.create_client(
            Empty, self.get_parameter('pause_service').value)
        self.resume_client = self.create_client(
            Empty, self.get_parameter('resume_service').value)
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info(
            f'SLAM 管理器就绪（当前{"开启" if self.enabled else "关闭"}）；'
            f'地图保存目录：{self.map_dir}')

    @staticmethod
    def resolve_map_dir(configured):
        if configured:
            return configured
        return default_map_dir()

    def call_service(self, client):
        if not client.service_is_ready():
            return
        future = client.call_async(Empty.Request())
        future.add_done_callback(
            lambda f: self.log_service_failure(f, client.srv_name))

    def log_service_failure(self, future, srv_name):
        """记录暂停/恢复服务的调用失败，否则操作静默失效、用户无从得知。"""
        exc = future.exception()
        if exc is not None:
            self.get_logger().error(f'调用服务 {srv_name} 失败：{exc}')

    def make_camera_info(self, header, frame_id):
        focal = (self.width / 2.0) / math.tan(self.hfov / 2.0)
        cx = self.width / 2.0
        cy = self.height / 2.0
        info = CameraInfo()
        info.header = header
        info.header.frame_id = frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0,
                  0.0]
        return info

    def on_image(self, msg):
        if not self.enabled:
            return
        msg.header.frame_id = self.camera_frame
        self.image_publisher.publish(msg)
        self.info_publisher.publish(
            self.make_camera_info(msg.header, self.camera_frame))

    def on_depth(self, msg):
        if not self.enabled:
            return
        msg.header.frame_id = self.camera_frame
        self.depth_publisher.publish(msg)
        self.depth_info_publisher.publish(
            self.make_camera_info(msg.header, self.camera_frame))

    def on_map(self, msg):
        self.latest_map = msg

    def on_cloud(self, msg):
        self.latest_cloud = msg

    def on_enable(self, msg):
        target = bool(msg.data)
        if target == self.enabled:
            return
        self.enabled = target
        if self.enabled:
            self.call_service(self.resume_client)
            self.get_logger().info('SLAM 建图已开启')
        else:
            self.call_service(self.pause_client)
            saved = self.save_map()
            if saved:
                self.get_logger().info(
                    'SLAM 建图已关闭，地图已保存：' + '、'.join(saved))
            else:
                self.get_logger().info('SLAM 建图已关闭（暂无可保存的地图数据）')
        self.publish_state()

    def publish_state(self):
        self.state_publisher.publish(Bool(data=self.enabled))

    def on_save_map(self, request, response):
        saved = self.save_map()
        response.success = bool(saved)
        response.message = '、'.join(saved) if saved else '暂无可保存的地图数据'
        return response

    def save_map(self):
        os.makedirs(self.map_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        saved = []
        if self.latest_map is not None and self.latest_map.info.width > 0:
            base = os.path.join(self.map_dir, f'maze_map_{stamp}')
            if self.save_occupancy_grid(self.latest_map, base):
                saved.append(base + '.yaml')
        if self.latest_cloud is not None and point_cloud2 is not None:
            path = os.path.join(self.map_dir, f'maze_cloud_{stamp}.pcd')
            if self.save_point_cloud(self.latest_cloud, path):
                saved.append(path)
        return saved

    def save_occupancy_grid(self, grid, base):
        try:
            width = grid.info.width
            height = grid.info.height
            with open(base + '.pgm', 'wb') as pgm:
                pgm.write(b'P5\n%d %d\n255\n' % (width, height))
                data = bytearray()
                for value in grid.data:
                    if value < 0:
                        data.append(205)
                    elif value >= 65:
                        data.append(0)
                    else:
                        data.append(254)
                pgm.write(bytes(data))
            origin = grid.info.origin
            yaw = math.atan2(2.0 * (origin.orientation.w * origin.orientation.z
                                    + origin.orientation.x * origin.orientation.y),
                             1.0 - 2.0 * (origin.orientation.y ** 2
                                          + origin.orientation.z ** 2))
            with open(base + '.yaml', 'w') as yaml:
                yaml.write(f'image: {os.path.basename(base)}.pgm\n')
                yaml.write(f'resolution: {grid.info.resolution}\n')
                yaml.write(f'origin: [{origin.position.x}, '
                           f'{origin.position.y}, {yaw}]\n')
                yaml.write('negate: 0\n')
                yaml.write('occupied_thresh: 0.65\n')
                yaml.write('free_thresh: 0.196\n')
            return True
        except Exception as exc:
            self.get_logger().error(f'保存栅格地图失败: {exc}')
            return False

    def save_point_cloud(self, cloud, path):
        try:
            fields = [field.name for field in cloud.fields]
            names = [name for name in ('x', 'y', 'z', 'intensity', 'rgb')
                     if name in fields]
            points = list(point_cloud2.read_points(cloud, field_names=names,
                                                   skip_nans=True))
            with open(path, 'w') as pcd:
                pcd.write('# .PCD v0.7 - Point Cloud Data file format\n')
                pcd.write('VERSION 0.7\n')
                pcd.write('FIELDS ' + ' '.join(names) + '\n')
                pcd.write('SIZE ' + ' '.join(['4'] * len(names)) + '\n')
                pcd.write('TYPE ' + ' '.join(['F'] * len(names)) + '\n')
                pcd.write('COUNT ' + ' '.join(['1'] * len(names)) + '\n')
                pcd.write(f'WIDTH {len(points)}\nHEIGHT 1\n')
                pcd.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                pcd.write(f'POINTS {len(points)}\nDATA ascii\n')
                for point in points:
                    values = point if isinstance(point, (list, tuple)) else list(point)
                    pcd.write(' '.join(f'{value:.4f}' for value in values) + '\n')
            return True
        except Exception as exc:
            self.get_logger().error(f'保存点云失败: {exc}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass
    finally:
        if node.latest_map is not None or node.latest_cloud is not None:
            node.save_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()