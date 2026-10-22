#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int64, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .core import (
    correction_frame_to_raw_odom,
    integrate_encoder_step,
    quaternion_from_yaw,
    recenter_heading,
    update_progressive_heading,
    wrap_angle,
    yaw_from_quaternion,
)


class BnoEncoderOdometry(Node):
    """Odométrie locale: distance encodeurs + cap BNO055.

    La différence de roues n'est jamais utilisée comme cap physique. Pendant une
    rotation sur place commandée, les ticks sont consommés mais x/y sont figés.
    """

    def __init__(self) -> None:
        super().__init__('ardubot_bno_encoder_odometry')

        defaults = {
            'imu_topic': '/imu',
            'left_ticks_topic': '/left_ticks',
            'right_ticks_topic': '/right_ticks',
            'cmd_vel_topic': '/cmd_vel',
            'raw_odom_topic': '/odom',
            'output_odom_topic': '/odom_bno_encoder',
            'path_topic': '/bno_encoder_path',
            'mode_topic': '/bno_encoder_odometry/mode',
            'joint_states_topic': '/joint_states',
            'odom_frame': 'odom_bno',
            'base_frame': 'base_link',
            'raw_odom_frame': 'odom',
            'tf_time_offset_s': 0.0,
            'ticks_per_revolution': 348.0,
            'wheel_radius_left_m': 0.015,
            'wheel_radius_right_m': 0.015,
            'wheel_base_m': 0.0784,
            'left_joint_direction': 1.0,
            'right_joint_direction': 1.0,
            'publish_rate_hz': 30.0,
            'sensor_timeout_s': 0.75,
            'cmd_timeout_s': 0.50,
            'post_command_grace_s': 0.25,
            'translation_linear_threshold_m_s': 0.010,
            'rotation_linear_threshold_m_s': 0.015,
            'rotation_angular_threshold_rad_s': 0.030,
            'gyro_filter_alpha': 0.25,
            'quaternion_time_constant_s': 8.0,
            'quaternion_weight_scale': 0.12,
            'post_rotation_recenter': True,
            'post_rotation_recenter_max_deg': 15.0,
            'post_rotation_settle_s': 0.25,
            'max_tick_jump': 10000,
            'publish_path': True,
            'path_max_poses': 2500,
            'path_min_step_m': 0.01,
            'pose_xy_variance': 0.0025,
            'pose_yaw_variance': 0.0012,
            'twist_linear_variance': 0.01,
            'twist_angular_variance': 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value

        self.imu_topic = str(p('imu_topic'))
        self.left_ticks_topic = str(p('left_ticks_topic'))
        self.right_ticks_topic = str(p('right_ticks_topic'))
        self.cmd_vel_topic = str(p('cmd_vel_topic'))
        self.raw_odom_topic = str(p('raw_odom_topic'))
        self.output_odom_topic = str(p('output_odom_topic'))
        self.path_topic = str(p('path_topic'))
        self.mode_topic = str(p('mode_topic'))
        self.joint_states_topic = str(p('joint_states_topic'))
        self.odom_frame = str(p('odom_frame'))
        self.base_frame = str(p('base_frame'))
        self.raw_odom_frame = str(p('raw_odom_frame'))
        self.tf_time_offset = float(p('tf_time_offset_s'))
        self.tpr = max(float(p('ticks_per_revolution')), 1.0)
        self.radius_left = float(p('wheel_radius_left_m'))
        self.radius_right = float(p('wheel_radius_right_m'))
        self.wheel_base = float(p('wheel_base_m'))
        self.left_joint_direction = self.sign_nonzero(float(p('left_joint_direction')))
        self.right_joint_direction = self.sign_nonzero(float(p('right_joint_direction')))
        self.sensor_timeout = float(p('sensor_timeout_s'))
        self.cmd_timeout = float(p('cmd_timeout_s'))
        self.command_grace = float(p('post_command_grace_s'))
        self.translation_threshold = float(p('translation_linear_threshold_m_s'))
        self.rotation_linear_threshold = float(p('rotation_linear_threshold_m_s'))
        self.rotation_angular_threshold = float(p('rotation_angular_threshold_rad_s'))
        self.gyro_alpha = max(0.0, min(1.0, float(p('gyro_filter_alpha'))))
        self.heading_tau = float(p('quaternion_time_constant_s'))
        self.heading_weight_scale = float(p('quaternion_weight_scale'))
        self.enable_recenter = bool(p('post_rotation_recenter'))
        self.recenter_limit = math.radians(abs(float(p('post_rotation_recenter_max_deg'))))
        self.rotation_settle = float(p('post_rotation_settle_s'))
        self.max_tick_jump = int(p('max_tick_jump'))
        self.publish_path_enabled = bool(p('publish_path'))
        self.path_max_poses = max(10, int(p('path_max_poses')))
        self.path_min_step = max(0.0, float(p('path_min_step_m')))
        self.pose_xy_variance = float(p('pose_xy_variance'))
        self.pose_yaw_variance = float(p('pose_yaw_variance'))
        self.twist_linear_variance = float(p('twist_linear_variance'))
        self.twist_angular_variance = float(p('twist_angular_variance'))

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 5)
        self.joint_state_pub = self.create_publisher(JointState, self.joint_states_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)
        self.create_subscription(Int64, self.left_ticks_topic, self.left_tick_callback, 100)
        self.create_subscription(Int64, self.right_ticks_topic, self.right_tick_callback, 100)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 20)
        self.create_subscription(Odometry, self.raw_odom_topic, self.raw_odom_callback, 20)
        self.create_service(Trigger, '~/reset', self.reset_callback)

        period = 1.0 / max(float(p('publish_rate_hz')), 1.0)
        self.create_timer(period, self.timer_callback)

        self.left_ticks: Optional[int] = None
        self.right_ticks: Optional[int] = None
        self.prev_left_ticks: Optional[int] = None
        self.prev_right_ticks: Optional[int] = None
        self.left_time: Optional[float] = None
        self.right_time: Optional[float] = None

        # Suivi indépendant pour /joint_states. On ne publie qu'après réception
        # d'une nouvelle paire gauche+droite, afin d'éviter les messages
        # artificiels vitesse réelle -> 0 -> vitesse réelle entre deux trames ENC.
        self.joint_prev_left_ticks: Optional[int] = None
        self.joint_prev_right_ticks: Optional[int] = None
        self.left_tick_updated = False
        self.right_tick_updated = False
        self.last_joint_state_time: Optional[float] = None

        self.raw_quaternion_yaw: Optional[float] = None
        self.fused_heading_unwrapped: Optional[float] = None
        self.heading_origin_unwrapped: Optional[float] = None
        self.previous_relative_yaw: Optional[float] = None
        self.gyro_filtered = 0.0
        self.imu_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None

        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.cmd_time: Optional[float] = None
        self.raw_linear = 0.0
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0
        self.raw_stamp = None
        self.raw_odom_time: Optional[float] = None
        self.last_motion_mode = 'STOP'
        self.last_motion_direction = 0.0
        self.last_nonzero_cmd_time: Optional[float] = None
        # Les encodeurs ne donnent que des impulsions positives : le sens réel de
        # chaque roue est reconstruit à partir de /cmd_vel. Ces signes sont
        # conservés pendant le court freinage après une commande nulle.
        self.last_left_wheel_sign = 0.0
        self.last_right_wheel_sign = 0.0
        self.rotation_end_time: Optional[float] = None
        self.recenter_pending = False

        self.x = 0.0
        self.y = 0.0
        self.path_length = 0.0
        self.linear_velocity = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.last_timer_time: Optional[float] = None
        self.last_mode = ''
        self.initialized_logged = False
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.last_path_x = 0.0
        self.last_path_y = 0.0

        self.get_logger().info(
            'Odométrie BNO-encodeurs prête : '
            f'{self.left_ticks_topic}+{self.right_ticks_topic}+{self.imu_topic} '
            f'-> {self.output_odom_topic}, TF de correction {self.odom_frame}->{self.raw_odom_frame}; '
            f'la chaîne finale {self.odom_frame}->{self.raw_odom_frame}->{self.base_frame} '
            'représente la pose BNO-encodeurs; différentiel de roues exclu du cap; '
            f'rotations roues publiées sur {self.joint_states_topic} à partir des encodeurs.'
        )

    def left_tick_callback(self, msg: Int64) -> None:
        self.left_ticks = int(msg.data)
        self.left_time = time.monotonic()
        self.left_tick_updated = True

    def right_tick_callback(self, msg: Int64) -> None:
        self.right_ticks = int(msg.data)
        self.right_time = time.monotonic()
        self.right_tick_updated = True

    @staticmethod
    def sign_nonzero(value: float, epsilon: float = 1.0e-4) -> float:
        if abs(value) <= epsilon:
            return 0.0
        return 1.0 if value > 0.0 else -1.0

    def wheel_command_signs(self, linear: float, angular: float) -> tuple[float, float]:
        """Sens des roues d'un différentiel à partir de la commande cinématique.

        Les compteurs encodeurs ArduBot sont non signés : ils comptent les fronts.
        /cmd_vel sert donc uniquement à reconstruire le signe de rotation visuelle
        de chaque roue. Le BNO reste seul responsable du cap du robot.
        """
        left_cmd = linear - 0.5 * self.wheel_base * angular
        right_cmd = linear + 0.5 * self.wheel_base * angular
        return self.sign_nonzero(left_cmd), self.sign_nonzero(right_cmd)

    def current_wheel_signs(self, now: float) -> tuple[float, float]:
        if self.command_fresh(now):
            left_sign, right_sign = self.wheel_command_signs(
                self.cmd_linear, self.cmd_angular
            )
            command_is_zero = left_sign == 0.0 and right_sign == 0.0
            if (command_is_zero and self.last_nonzero_cmd_time is not None
                    and now - self.last_nonzero_cmd_time <= self.command_grace):
                return self.last_left_wheel_sign, self.last_right_wheel_sign
            return left_sign, right_sign

        if (self.last_nonzero_cmd_time is not None
                and now - self.last_nonzero_cmd_time <= self.command_grace):
            return self.last_left_wheel_sign, self.last_right_wheel_sign

        # Même repli que l'odométrie pour une petite translation résiduelle.
        raw_fresh = (
            self.raw_odom_time is not None
            and now - self.raw_odom_time <= self.sensor_timeout
        )
        if raw_fresh and abs(self.raw_linear) >= self.translation_threshold:
            sign = self.sign_nonzero(self.raw_linear)
            return sign, sign

        return 0.0, 0.0

    def cmd_callback(self, msg: Twist) -> None:
        now = time.monotonic()
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)
        self.cmd_time = now

        left_sign, right_sign = self.wheel_command_signs(
            self.cmd_linear, self.cmd_angular
        )
        if left_sign != 0.0 or right_sign != 0.0:
            self.last_left_wheel_sign = left_sign
            self.last_right_wheel_sign = right_sign

        if abs(self.cmd_linear) >= self.translation_threshold or abs(self.cmd_angular) >= self.rotation_angular_threshold:
            self.last_nonzero_cmd_time = now

    def raw_odom_callback(self, msg: Odometry) -> None:
        self.raw_linear = float(msg.twist.twist.linear.x)
        self.raw_x = float(msg.pose.pose.position.x)
        self.raw_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.raw_yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        if msg.header.frame_id:
            self.raw_odom_frame = str(msg.header.frame_id)
        self.raw_stamp = msg.header.stamp
        self.raw_odom_time = time.monotonic()

    def imu_callback(self, msg: Imu) -> None:
        now = time.monotonic()
        q = msg.orientation
        raw_yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        gyro = float(msg.angular_velocity.z)
        if self.last_imu_time is None:
            dt = 0.0
            self.gyro_filtered = gyro
        else:
            dt = now - self.last_imu_time
            self.gyro_filtered = self.gyro_alpha * gyro + (1.0 - self.gyro_alpha) * self.gyro_filtered
        self.last_imu_time = now
        self.imu_time = now
        self.raw_quaternion_yaw = raw_yaw

        update = update_progressive_heading(
            self.fused_heading_unwrapped,
            raw_yaw,
            self.gyro_filtered,
            dt,
            self.heading_tau,
            self.heading_weight_scale,
        )
        self.fused_heading_unwrapped = update.fused_unwrapped
        if self.heading_origin_unwrapped is None:
            self.heading_origin_unwrapped = self.fused_heading_unwrapped
            self.previous_relative_yaw = 0.0

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.reset_pose()
        response.success = True
        response.message = 'Origine odom_bno remise à zéro.'
        return response

    def reset_pose(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.path_length = 0.0
        self.linear_velocity = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.joint_prev_left_ticks = self.left_ticks
        self.joint_prev_right_ticks = self.right_ticks
        self.left_tick_updated = False
        self.right_tick_updated = False
        self.last_joint_state_time = None
        if self.fused_heading_unwrapped is not None:
            self.heading_origin_unwrapped = self.fused_heading_unwrapped
        self.previous_relative_yaw = 0.0
        self.path.poses.clear()
        self.last_path_x = 0.0
        self.last_path_y = 0.0
        self.get_logger().warn('Origine odométrique BNO-encodeurs remise à zéro.')

    def source_fresh(self, stamp: Optional[float], now: float) -> bool:
        return stamp is not None and now - stamp <= self.sensor_timeout

    def command_fresh(self, now: float) -> bool:
        return self.cmd_time is not None and now - self.cmd_time <= self.cmd_timeout

    def classify_motion(self, now: float) -> tuple[str, float]:
        if self.command_fresh(now):
            linear = self.cmd_linear
            angular = self.cmd_angular
            command_is_zero = (
                abs(linear) < self.translation_threshold
                and abs(angular) < self.rotation_angular_threshold
            )
            if (command_is_zero and self.last_nonzero_cmd_time is not None
                    and now - self.last_nonzero_cmd_time <= self.command_grace):
                # Consomme le petit déplacement de freinage avec le dernier mode.
                mode = self.last_motion_mode
                direction = self.last_motion_direction
            elif abs(linear) <= self.rotation_linear_threshold and abs(angular) >= self.rotation_angular_threshold:
                mode = 'ROTATION_FREEZE_XY'
                direction = 0.0
            elif abs(linear) >= self.translation_threshold:
                mode = 'TRANSLATION_BNO_ENCODER'
                direction = 1.0 if linear > 0.0 else -1.0
            else:
                mode = 'STOP'
                direction = 0.0
        elif self.last_nonzero_cmd_time is not None and now - self.last_nonzero_cmd_time <= self.command_grace:
            mode = self.last_motion_mode
            direction = self.last_motion_direction
        else:
            # Repli uniquement pour une éventuelle translation résiduelle. Le yaw
            # différentiel brut n'est jamais utilisé pour détecter une rotation.
            raw_fresh = self.raw_odom_time is not None and now - self.raw_odom_time <= self.sensor_timeout
            if raw_fresh and abs(self.raw_linear) >= self.translation_threshold:
                mode = 'TRANSLATION_RAW_SIGN_FALLBACK'
                direction = 1.0 if self.raw_linear > 0.0 else -1.0
            else:
                mode = 'STOP'
                direction = 0.0

        if self.last_motion_mode == 'ROTATION_FREEZE_XY' and mode != 'ROTATION_FREEZE_XY':
            self.rotation_end_time = now
            self.recenter_pending = True
        self.last_motion_mode = mode
        self.last_motion_direction = direction
        return mode, direction

    def publish_joint_state(
        self,
        stamp,
        delta_left: int,
        delta_right: int,
        now_mono: float,
        dt: float,
    ) -> None:
        """Publie la rotation visuelle réelle des roues.

        L'amplitude vient des ticks encodeurs. Leur signe, absent du matériel, est
        déduit de /cmd_vel roue par roue. Cette publication n'intervient jamais
        dans le calcul du cap BNO ni dans la correction odom_bno->odom.
        """
        left_sign, right_sign = self.current_wheel_signs(now_mono)

        left_delta_angle = (
            self.left_joint_direction * left_sign
            * float(delta_left) * 2.0 * math.pi / self.tpr
        )
        right_delta_angle = (
            self.right_joint_direction * right_sign
            * float(delta_right) * 2.0 * math.pi / self.tpr
        )

        self.left_wheel_angle += left_delta_angle
        self.right_wheel_angle += right_delta_angle

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_wheel_angle, self.right_wheel_angle]

        if dt > 1.0e-4:
            msg.velocity = [left_delta_angle / dt, right_delta_angle / dt]
        else:
            msg.velocity = [0.0, 0.0]

        self.joint_state_pub.publish(msg)

    def publish_joint_state_if_new_encoder_pair(self, stamp, now_mono: float) -> None:
        """Publie une seule fois par nouvelle paire d'échantillons encodeurs.

        Le timer BNO tourne plus vite que les trames ENC. Sans cette garde, la
        même position était republiée entre deux trames avec velocity=[0, 0],
        créant artificiellement une alternance vitesse réelle / vitesse nulle.
        """
        if self.left_ticks is None or self.right_ticks is None:
            return

        # Attendre que les deux côtés aient reçu au moins un nouvel échantillon.
        if not (self.left_tick_updated and self.right_tick_updated):
            return

        # Première paire : initialise l'origine visuelle des roues à 0 rad.
        if self.joint_prev_left_ticks is None or self.joint_prev_right_ticks is None:
            self.joint_prev_left_ticks = int(self.left_ticks)
            self.joint_prev_right_ticks = int(self.right_ticks)
            self.left_tick_updated = False
            self.right_tick_updated = False
            self.last_joint_state_time = now_mono
            self.publish_joint_state(stamp, 0, 0, now_mono, 0.0)
            return

        delta_left = int(self.left_ticks) - int(self.joint_prev_left_ticks)
        delta_right = int(self.right_ticks) - int(self.joint_prev_right_ticks)

        # Consommer la paire même en cas de saut invalide, pour ne pas reporter
        # un énorme delta sur l'échantillon suivant.
        self.joint_prev_left_ticks = int(self.left_ticks)
        self.joint_prev_right_ticks = int(self.right_ticks)
        self.left_tick_updated = False
        self.right_tick_updated = False

        if abs(delta_left) > self.max_tick_jump or abs(delta_right) > self.max_tick_jump:
            self.last_joint_state_time = now_mono
            return

        dt = (
            0.0
            if self.last_joint_state_time is None
            else now_mono - self.last_joint_state_time
        )
        self.last_joint_state_time = now_mono
        self.publish_joint_state(
            stamp, delta_left, delta_right, now_mono, dt
        )

    def maybe_recenter_after_rotation(self, now: float, mode: str) -> None:
        if not self.enable_recenter or not self.recenter_pending:
            return
        if mode != 'STOP' or self.rotation_end_time is None:
            return
        if now - self.rotation_end_time < self.rotation_settle:
            return
        if self.fused_heading_unwrapped is None or self.raw_quaternion_yaw is None:
            return
        self.fused_heading_unwrapped, applied = recenter_heading(
            self.fused_heading_unwrapped,
            self.raw_quaternion_yaw,
            self.recenter_limit,
        )
        self.recenter_pending = False
        self.get_logger().info(
            f'Recalage quaternion après rotation : {math.degrees(applied):+.2f}°'
        )

    def current_relative_yaw(self) -> Optional[float]:
        if self.fused_heading_unwrapped is None or self.heading_origin_unwrapped is None:
            return None
        return self.fused_heading_unwrapped - self.heading_origin_unwrapped

    def measurement_stamp(self):
        """Use the Raspberry odometry clock so TF and /scan share a timeline."""
        if self.raw_stamp is not None:
            sec = int(self.raw_stamp.sec)
            nanosec = int(self.raw_stamp.nanosec)
            if sec != 0 or nanosec != 0:
                try:
                    return (Time.from_msg(self.raw_stamp) + Duration(seconds=self.tf_time_offset)).to_msg()
                except (ValueError, OverflowError):
                    pass
        return self.get_clock().now().to_msg()

    def timer_callback(self) -> None:
        now_mono = time.monotonic()
        now_ros = self.measurement_stamp()
        dt = 0.0 if self.last_timer_time is None else now_mono - self.last_timer_time
        self.last_timer_time = now_mono

        sensors_ready = (
            self.left_ticks is not None
            and self.right_ticks is not None
            and self.current_relative_yaw() is not None
        )
        sensors_fresh = (
            self.source_fresh(self.left_time, now_mono)
            and self.source_fresh(self.right_time, now_mono)
            and self.source_fresh(self.imu_time, now_mono)
        )

        if not sensors_ready:
            self.publish_mode('WAITING_SENSORS')
            return

        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.previous_relative_yaw = self.current_relative_yaw()
            self.publish_joint_state_if_new_encoder_pair(now_ros, now_mono)
            self.publish_odometry(now_ros, 'INITIALIZED', sensors_fresh)
            if not self.initialized_logged:
                self.initialized_logged = True
                self.get_logger().info('Origine odom_bno initialisée à x=0, y=0, yaw=0.')
            return

        delta_left = int(self.left_ticks) - int(self.prev_left_ticks)
        delta_right = int(self.right_ticks) - int(self.prev_right_ticks)
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        if abs(delta_left) > self.max_tick_jump or abs(delta_right) > self.max_tick_jump:
            self.linear_velocity = 0.0
            self.publish_odometry(now_ros, 'TICK_JUMP_IGNORED', sensors_fresh)
            return

        mode, direction = self.classify_motion(now_mono)
        self.maybe_recenter_after_rotation(now_mono, mode)
        current_yaw = self.current_relative_yaw()
        if current_yaw is None:
            return
        previous_yaw = current_yaw if self.previous_relative_yaw is None else self.previous_relative_yaw

        dl = float(delta_left) * (2.0 * math.pi * self.radius_left) / self.tpr
        dr = float(delta_right) * (2.0 * math.pi * self.radius_right) / self.tpr
        centre_unsigned = 0.5 * (abs(dl) + abs(dr))

        # Rotation des roues pour RobotModel/RViz : indépendante du cap BNO.
        # Publication uniquement à la réception d'une nouvelle paire ENC.
        self.publish_joint_state_if_new_encoder_pair(now_ros, now_mono)

        translate = sensors_fresh and mode.startswith('TRANSLATION')
        pose = integrate_encoder_step(
            self.x,
            self.y,
            self.path_length,
            centre_unsigned,
            direction,
            previous_yaw,
            current_yaw,
            translate,
        )
        moved = math.hypot(pose.x - self.x, pose.y - self.y)
        self.x, self.y, self.path_length = pose.x, pose.y, pose.path_length
        self.linear_velocity = (direction * centre_unsigned / dt) if translate and dt > 1.0e-4 else 0.0
        self.previous_relative_yaw = current_yaw

        if not sensors_fresh:
            mode = 'SENSORS_STALE_HOLD'
            self.linear_velocity = 0.0

        self.publish_odometry(now_ros, mode, sensors_fresh)
        path_step = math.hypot(self.x - self.last_path_x, self.y - self.last_path_y)
        if self.publish_path_enabled and path_step >= self.path_min_step:
            self.append_path_pose(now_ros, current_yaw)

    def publish_odometry(self, stamp, mode: str, sensors_fresh: bool) -> None:
        yaw_rel = self.current_relative_yaw()
        if yaw_rel is None:
            return
        qx, qy, qz, qw = quaternion_from_yaw(wrap_angle(yaw_rel))

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.linear_velocity if sensors_fresh else 0.0
        msg.twist.twist.angular.z = self.gyro_filtered if sensors_fresh else 0.0
        msg.pose.covariance[0] = self.pose_xy_variance
        msg.pose.covariance[7] = self.pose_xy_variance
        msg.pose.covariance[35] = self.pose_yaw_variance
        msg.twist.covariance[0] = self.twist_linear_variance
        msg.twist.covariance[35] = self.twist_angular_variance
        self.odom_pub.publish(msg)

        # Keep the Raspberry raw TF odom->base_link and publish only the
        # correction odom_bno->odom. This gives base_link a single parent and
        # makes the composed pose equal to the BNO-encoder pose.
        if self.raw_odom_time is not None:
            correction = correction_frame_to_raw_odom(
                self.x, self.y, wrap_angle(yaw_rel),
                self.raw_x, self.raw_y, self.raw_yaw,
            )
            cqx, cqy, cqz, cqw = quaternion_from_yaw(correction.yaw)
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.raw_odom_frame
            tf.transform.translation.x = correction.x
            tf.transform.translation.y = correction.y
            tf.transform.rotation.x = cqx
            tf.transform.rotation.y = cqy
            tf.transform.rotation.z = cqz
            tf.transform.rotation.w = cqw
            self.tf_broadcaster.sendTransform(tf)
        self.publish_mode(mode)

    def append_path_pose(self, stamp, yaw_rel: float) -> None:
        qx, qy, qz, qw = quaternion_from_yaw(wrap_angle(yaw_rel))
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.odom_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.path_max_poses:
            self.path.poses = self.path.poses[-self.path_max_poses:]
        self.path_pub.publish(self.path)
        self.last_path_x = self.x
        self.last_path_y = self.y

    def publish_mode(self, mode: str) -> None:
        if mode == self.last_mode:
            return
        self.last_mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Mode odométrie : {mode}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BnoEncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Humble can throw this while a bridged subscription is being torn down.
        if 'Unable to convert call argument to Python object' not in str(exc):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
