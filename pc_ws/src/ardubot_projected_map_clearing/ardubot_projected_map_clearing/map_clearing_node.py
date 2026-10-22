#!/usr/bin/env python3
"""ArduBot: 2D obstacle projection with FOV-based virtual free-space clearing.

No LaserScan subscription. This publishes a VISUALIZATION/EXPERIMENT map only;
/map and /map_cleared, Nav2, SLAM, and the Raspberry are not modified.
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformException, TransformListener

from ardubot_projected_map_clearing.grid import PersistentGrid


class ProjectedMapClearingNode(Node):
    def __init__(self):
        super().__init__('ardubot_projected_map_clearing')
        defaults = {
            'map_topic': '/projected_map',
            'output_map_topic': '/projected_map_cleared',
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'fov_degrees': 62.8,
            'clear_range': 2.5,
            'camera_forward_offset': 0.085,
            'tf_timeout': 0.08,
            'source_obstacle_threshold': 99,
            'source_free_max_value': 0,
            'map_resolution': 0.05,
            'initial_size_m': 16.0,
            'expand_step_m': 2.0,
            'publish_period': 0.25,
        }
        for name, default_value in defaults.items():
            self.declare_parameter(name, default_value)
        par = lambda name: self.get_parameter(name).value
        self.map_topic = str(par('map_topic'))
        self.output_topic = str(par('output_map_topic'))
        self.map_frame = str(par('map_frame'))
        self.robot_frame = str(par('robot_frame'))
        self.fov_degrees = float(par('fov_degrees'))
        self.clear_range = float(par('clear_range'))
        self.camera_offset = float(par('camera_forward_offset'))
        self.tf_timeout = float(par('tf_timeout'))
        self.obstacle_threshold = int(par('source_obstacle_threshold'))
        self.free_max = int(par('source_free_max_value'))
        self.publish_period = float(par('publish_period'))

        if not (0 < self.fov_degrees <= 180):
            raise ValueError('fov_degrees must be in (0, 180]')
        if not (0 < self.clear_range <= 20):
            raise ValueError('clear_range must be in (0, 20]')
        if not (-1 <= self.camera_offset <= 1):
            raise ValueError('camera_forward_offset must be between -1 and 1 m')
        if self.publish_period < 0.1:
            raise ValueError('publish_period must be >= 0.1 s')

        self.map_grid = PersistentGrid(
            resolution=float(par('map_resolution')),
            initial_size_m=float(par('initial_size_m')),
            expand_step_m=float(par('expand_step_m')),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_rejected_source_frame = None
        self.warned_tf = False

        # Subscription compatible with BEST_EFFORT and RELIABLE publishers.
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_source_map, input_qos)
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.output_topic, output_qos)
        self.timer = self.create_timer(self.publish_period, self.update_and_publish)

        self.get_logger().info(
            f'Projected 3D CURRENT obstacles + persistent FOV free-space: {self.map_topic} + virtual FOV '
            f'{self.fov_degrees:.1f} deg ahead of {self.robot_frame} '
            f'(camera offset {self.camera_offset:+.3f}m), '
            f'range {self.clear_range:.2f}m -> {self.output_topic}; '
            f'initial {self.map_grid.initial_cells * self.map_grid.resolution:.1f}m, '
            f'expand by {self.map_grid.expand_cells * self.map_grid.resolution:.1f}m, '
            f'resolution {self.map_grid.resolution:.3f}m. NO /scan subscription. '
            '/map and /map_cleared unchanged. Old obstacles are NOT accumulated.'
        )

    @staticmethod
    def yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_source_map(self, msg):
        if msg.header.frame_id != self.map_frame:
            if msg.header.frame_id != self.last_rejected_source_frame:
                self.get_logger().error(
                    f'Expected {self.map_frame!r}, received {msg.header.frame_id!r}. '
                    'No unsafe merging across frames.')
                self.last_rejected_source_frame = msg.header.frame_id
            return
        if msg.info.resolution <= 0.0 or not msg.info.width or not msg.info.height:
            return
        orient = msg.info.origin.orientation
        if (abs(orient.x) > 1e-4 or abs(orient.y) > 1e-4 or
            abs(orient.z) > 1e-4 or abs(abs(orient.w) - 1.0) > 1e-4):
            self.get_logger().warning(
                'Rotated OccupancyGrid origin is not supported; skipping map.',
                throttle_duration_sec=5.0)
            return
        before = (self.map_grid.width, self.map_grid.height)
        try:
            self.map_grid.merge_source(
                float(msg.info.origin.position.x), float(msg.info.origin.position.y),
                int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                msg.data, obstacle_threshold=self.obstacle_threshold,
                free_max_value=self.free_max,
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid /projected_map: {exc}')
            return
        after = (self.map_grid.width, self.map_grid.height)
        if after != before:
            self.get_logger().info(f'Persistent grid size: {before} -> {after} cells')

    def update_and_publish(self):
        g = self.map_grid
        if not g.initialized:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
            self.warned_tf = False
        except TransformException as exc:
            if not self.warned_tf:
                self.get_logger().warn(
                    f'Cannot clear FOV without TF {self.map_frame} <- '
                    f'{self.robot_frame}: {exc}')
                self.warned_tf = True
            # Still publish the incoming /projected_map obstacle/free history.
            self.publish_grid()
            return

        p = tf.transform.translation
        yaw = self.yaw(tf.transform.rotation)
        # The measured base_link -> sensor_frame is x=+0.085m, y=0.0m,
        # while optical forward is sensor Z. Never use sensor_frame yaw=90 deg
        # as robot heading: here base_link +X is the forward direction.
        ox = float(p.x) + self.camera_offset * math.cos(yaw)
        oy = float(p.y) + self.camera_offset * math.sin(yaw)
        g.ensure_bounds(ox - self.clear_range, oy - self.clear_range,
                        ox + self.clear_range, oy + self.clear_range)
        g.clear_fov(ox, oy, yaw, self.fov_degrees, self.clear_range)
        self.publish_grid()

    def publish_grid(self):
        g = self.map_grid
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = g.resolution
        msg.info.width = g.width
        msg.info.height = g.height
        msg.info.origin.position.x = g.origin_x
        msg.info.origin.position.y = g.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = g.compose()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectedMapClearingNode()
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
