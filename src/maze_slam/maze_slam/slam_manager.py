#!/usr/bin/env python3
"""SLAM 数据转发与建图开关中枢节点。

在整个系统中的位置：
    Gazebo 相机（经 ros_gz_bridge）发布原始图像/深度 -> 本节点按开关转发到
    /slam/image、/slam/depth_image，并自行合成 CameraInfo -> rtabmap_slam 消费；
    rtabmap 产出的 /map（OccupancyGrid）与 /rtabmap/cloud_map（PointCloud2）
    由本节点缓存，供停止建图或收到保存请求时落盘。

为什么不直接让 rtabmap 订阅 Gazebo 原始话题：
    * ros_gz_bridge 只桥接 image/depth，不桥接 camera_info，而 rtabmap 的视觉
      里程计必须知道内参，所以这里按 image_width/image_height/horizontal_fov
      自行合成 CameraInfo（见 make_camera_info）；
    * 需要一个统一的建图开关（/slam_enable）：开启时 resume rtabmap，关闭时
      pause 并立即保存地图，避免无人操作时数据库持续增长。

数据流：
    /camera/image_raw ─┐(仅 enabled 时转发)
    /camera/depth_image┘→ /slam/image、/slam/depth_image 及 CameraInfo → rtabmap
    rtabmap → /map、/rtabmap/cloud_map → 本节点缓存 → save_map() 落盘
    /slam_enable → enabled 状态 → /slam_state（周期上报，GUI 据此判断在线）
"""
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

# 点云逐点读取依赖 sensor_msgs_py；缺失时降级为“只保存栅格地图”，
# 不影响图像转发与建图主流程（见 save_point_cloud 的调用条件）
try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    point_cloud2 = None


class SlamManager(Node):
    """把 Gazebo 相机数据转发给 rtabmap，并管理建图开关与地图保存。

    状态机很简单：``enabled`` 为 True 时转发图像并 resume rtabmap；
    为 False 时停止转发、pause rtabmap，并尝试保存已积累的地图。
    节点周期性把 ``enabled`` 上报到 /slam_state，供 GUI 判断在线与状态。
    """

    def __init__(self):
        super().__init__('slam_manager')
        # 所有话题/服务与相机几何量都声明为参数，便于 launch 或命令行覆盖；
        # 默认值对应 spawn_maze.launch.py 的相机话题与 slam.launch.py 的 remap
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
        # width/height/hfov 用于合成 CameraInfo，必须与 Gazebo 相机配置一致
        self.hfov = float(self.get_parameter('horizontal_fov').value)
        # 建图默认关闭：避免一启动就凭空建图，由 GUI/命令行发 /slam_enable 开启
        self.enabled = bool(self.get_parameter('enabled').value)
        # map_dir 为空时走 resource_paths 的解析逻辑（见该模块的说明）
        self.map_dir = self.resolve_map_dir(self.get_parameter('map_dir').value)
        # 缓存最近一帧地图/点云：停止建图或收到保存请求时落盘。
        # 只需最新值，无需累积历史（rtabmap 输出的已是全图）
        self.latest_map = None
        self.latest_cloud = None

        # 订阅 Gazebo 相机：图像/深度体积大、频率高，用 BEST_EFFORT，
        # 下游来不及消费时直接丢旧帧，避免重传队列堆积把整条数据流堵死
        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        # 转发给 rtabmap 的话题用 RELIABLE：rtabmap 要求彩图与深度成对同步，
        # 传输丢帧会造成同步失败，这里宁可慢一点也要保证送达
        slam_qos = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        # 地图/点云是低频的“最终结果”，用 TRANSIENT_LOCAL + depth=1：
        # 后启动的订阅者（如 RViz、保存脚本）也能立刻拿到最后一帧，无需重发
        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # 转发输出：图像/深度，以及为它们配套合成的 CameraInfo
        self.image_publisher = self.create_publisher(Image, slam_image_topic,
                                                     slam_qos)
        self.depth_publisher = self.create_publisher(Image, slam_depth_topic,
                                                     slam_qos)
        # CameraInfo 数据量很小，用默认 QoS（depth 5、RELIABLE）即可
        self.info_publisher = self.create_publisher(CameraInfo,
                                                    camera_info_topic, 5)
        self.depth_info_publisher = self.create_publisher(CameraInfo,
                                                          depth_info_topic, 5)
        # 周期上报开关状态，让后启动的 GUI 也能同步当前值
        self.state_publisher = self.create_publisher(Bool, state_topic, 10)
        # 输入：Gazebo 桥接来的原始相机数据；enable：外部下发的建图开关
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.create_subscription(Image, depth_topic, self.on_depth, sensor_qos)
        self.create_subscription(Bool, enable_topic, self.on_enable, 10)
        # 输出：rtabmap 产出的栅格地图与点云（TRANSIENT_LOCAL，见 map_qos 说明）
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud,
                                 map_qos)
        # 主动保存服务（~/save_map 展开为 /slam_manager/save_map），
        # 用于在不关闭建图的情况下把当前地图落盘
        self.create_service(Trigger, '~/save_map', self.on_save_map)
        # rtabmap 的暂停/恢复服务。这里只创建 client、不阻塞等待：
        # rtabmap 可能比本节点后启动，调用时再用 service_is_ready() 判断
        self.pause_client = self.create_client(
            Empty, self.get_parameter('pause_service').value)
        self.resume_client = self.create_client(
            Empty, self.get_parameter('resume_service').value)
        # 1 Hz 上报状态：GUI 用“2 秒内是否收到”判断本节点在线
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info(
            f'SLAM 管理器就绪（当前{"开启" if self.enabled else "关闭"}）；'
            f'地图保存目录：{self.map_dir}')

    @staticmethod
    def resolve_map_dir(configured):
        """确定地图保存目录：显式参数优先，否则交给 resource_paths 解析。"""
        if configured:
            return configured
        return default_map_dir()

    def call_service(self, client):
        """异步调用 rtabmap 的 pause/resume 服务，不阻塞回调线程。

        服务未就绪时（rtabmap 还没启动完）直接跳过：此时 rtabmap 本就没在
        处理数据，无需暂停；其上线后的状态仍由后续的 enable 消息决定。
        """
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
        """按 width/height/hfov 合成 CameraInfo。

        ros_gz_bridge 不桥接 camera_info，而 rtabmap 需要内参才能做视觉里程计；
        这里假设像素为正方形、无畸变，由水平视场角反推焦距：
        focal = (width / 2) / tan(hfov / 2)，主点取图像中心。
        """
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
        """收到彩色图像：仅在建图开启时转发，并附带合成的 CameraInfo。

        统一改写 frame_id 为 camera_frame（与 URDF/TF 中的相机连杆一致），
        否则 rtabmap 查不到外参、视觉里程计无法工作。
        """
        if not self.enabled:
            return
        msg.header.frame_id = self.camera_frame
        self.image_publisher.publish(msg)
        # 每帧都配一份 CameraInfo：rtabmap 按时间戳把 rgb 与 camera_info 配对
        self.info_publisher.publish(
            self.make_camera_info(msg.header, self.camera_frame))

    def on_depth(self, msg):
        """收到深度图像：与彩色图同处理，转发到 rtabmap 的 depth 输入。"""
        if not self.enabled:
            return
        msg.header.frame_id = self.camera_frame
        self.depth_publisher.publish(msg)
        self.depth_info_publisher.publish(
            self.make_camera_info(msg.header, self.camera_frame))

    def on_map(self, msg):
        """缓存最新栅格地图，供 save_map() 落盘。"""
        self.latest_map = msg

    def on_cloud(self, msg):
        """缓存最新点云地图，供 save_map() 落盘。"""
        self.latest_cloud = msg

    def on_enable(self, msg):
        """处理 /slam_enable 开关：翻转建图状态并同步 rtabmap 的暂停/恢复。

        只在状态真正变化时动作，避免 GUI 周期性重发同一指令导致反复
        pause/resume（每次切换 rtabmap 都要重载数据库，代价不小）。
        """
        target = bool(msg.data)
        if target == self.enabled:
            return
        self.enabled = target
        if self.enabled:
            # 开启：恢复 rtabmap 处理数据
            self.call_service(self.resume_client)
            self.get_logger().info('SLAM 建图已开启')
        else:
            # 关闭：先暂停再保存，确保落盘的是本段建图的最终结果
            self.call_service(self.pause_client)
            saved = self.save_map()
            if saved:
                self.get_logger().info(
                    'SLAM 建图已关闭，地图已保存：' + '、'.join(saved))
            else:
                self.get_logger().info('SLAM 建图已关闭（暂无可保存的地图数据）')
        self.publish_state()

    def publish_state(self):
        """把当前建图开关状态发布到 /slam_state。"""
        self.state_publisher.publish(Bool(data=self.enabled))

    def on_save_map(self, request, response):
        """处理 ~/save_map 服务：保存当前地图，用保存结果填充应答。"""
        saved = self.save_map()
        response.success = bool(saved)
        response.message = '、'.join(saved) if saved else '暂无可保存的地图数据'
        return response

    def save_map(self):
        """把缓存的地图/点云各存一份，返回实际生成的文件路径列表。

        文件名带本地时间戳，多次保存互不覆盖；无数据时返回空列表，
        由调用方决定提示方式（服务返回 success=False 或打印日志）。
        """
        os.makedirs(self.map_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        saved = []
        # 栅格地图：width>0 才算有效（rtabmap 尚未产出时 info 为空）
        if self.latest_map is not None and self.latest_map.info.width > 0:
            base = os.path.join(self.map_dir, f'maze_map_{stamp}')
            if self.save_occupancy_grid(self.latest_map, base):
                saved.append(base + '.yaml')
        # 点云：sensor_msgs_py 不可用（point_cloud2 is None）时跳过，避免报错
        if self.latest_cloud is not None and point_cloud2 is not None:
            path = os.path.join(self.map_dir, f'maze_cloud_{stamp}.pcd')
            if self.save_point_cloud(self.latest_cloud, path):
                saved.append(path)
        return saved

    def save_occupancy_grid(self, grid, base):
        """把 OccupancyGrid 写成 ROS 地图服务器可读的 PGM + YAML 组合。

        灰度映射遵循 map_server 约定（0 黑=占据，255 白=空闲）：
        -1（未知）-> 205 灰，>=65（占据概率高）-> 0 黑，其余 -> 254 白；
        阈值与下方 YAML 的 occupied_thresh/free_thresh 保持一致。
        """
        try:
            width = grid.info.width
            height = grid.info.height
            # PGM 用二进制 P5 格式，第三行 255 为最大灰度值
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
            # map_server 的 origin 只接受偏航角，这里由四元数换算 yaw
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
        """把 PointCloud2 写成 ASCII 格式的 PCD（PCL 通用格式）。

        只挑出通用字段（x/y/z/intensity/rgb），未知字段直接忽略，
        避免下游 PCL/CloudCompare 因不认识的字段解析失败。
        """
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
    """节点入口：无论正常退出还是异常退出，都尽量保存尚未落盘的地图。"""
    rclpy.init(args=args)
    node = SlamManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, Exception):
        pass
    finally:
        # 兜底保存：Ctrl+C 直接退出时也把当前地图留下
        if node.latest_map is not None or node.latest_cloud is not None:
            node.save_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()