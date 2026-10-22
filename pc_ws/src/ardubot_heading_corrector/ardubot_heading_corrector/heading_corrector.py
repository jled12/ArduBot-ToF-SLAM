#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .control import clamp, profile_gains, wrap_angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingCorrector(Node):
    """Corrige seulement les lignes droites avant de publier /cmd_vel.

    Chaîne prévue :
      Nav2 -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
      -> ce nœud -> /cmd_vel -> bridge vers le Raspberry Pi.

    Courbes, rotations sur place et marche arrière sont transmises sans biais.
    """

    def __init__(self) -> None:
        super().__init__('ardubot_heading_corrector')

        self.declare_parameter('heading_profile', 'B2')
        profile = str(self.get_parameter('heading_profile').value).upper()
        gains = profile_gains(profile)

        # Ces paramètres peuvent être surchargés explicitement, mais leurs
        # valeurs par défaut proviennent du profil choisi.
        self.declare_parameter('heading_kcap', gains.kcap)
        self.declare_parameter('heading_kgyro', gains.kgyro)
        self.declare_parameter('heading_bias_rad_s', gains.bias_rad_s)

        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel_smoothed')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('mode_topic', '/heading_corrector/mode')

        self.declare_parameter('straight_min_linear_speed_m_s', 0.02)
        self.declare_parameter('straight_enter_angular_speed_rad_s', 0.015)
        self.declare_parameter('straight_exit_angular_speed_rad_s', 0.030)
        self.declare_parameter('max_heading_error_deg', 12.0)
        self.declare_parameter('max_straight_correction_rad_s', 0.16)
        self.declare_parameter('max_output_angular_speed_rad_s', 0.35)
        self.declare_parameter('imu_timeout_s', 0.50)
        self.declare_parameter('cmd_timeout_s', 0.50)
        self.declare_parameter('apply_forward_only', True)

        self.profile = profile
        self.kcap = float(self.get_parameter('heading_kcap').value)
        self.kgyro = float(self.get_parameter('heading_kgyro').value)
        self.bias = float(self.get_parameter('heading_bias_rad_s').value)

        self.input_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        self.output_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        mode_topic = str(self.get_parameter('mode_topic').value)

        self.min_linear = float(self.get_parameter('straight_min_linear_speed_m_s').value)
        self.enter_angular = float(
            self.get_parameter('straight_enter_angular_speed_rad_s').value
        )
        self.exit_angular = float(
            self.get_parameter('straight_exit_angular_speed_rad_s').value
        )
        self.max_heading_error = math.radians(
            float(self.get_parameter('max_heading_error_deg').value)
        )
        self.max_correction = float(
            self.get_parameter('max_straight_correction_rad_s').value
        )
        self.max_angular = float(
            self.get_parameter('max_output_angular_speed_rad_s').value
        )
        self.imu_timeout = float(self.get_parameter('imu_timeout_s').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        self.apply_forward_only = bool(self.get_parameter('apply_forward_only').value)

        if self.enter_angular < 0.0 or self.exit_angular <= self.enter_angular:
            raise ValueError(
                'Il faut 0 <= straight_enter_angular_speed_rad_s '
                '< straight_exit_angular_speed_rad_s.'
            )

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.mode_pub = self.create_publisher(String, mode_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 20)
        self.create_timer(0.05, self.watchdog_callback)

        self.yaw: Optional[float] = None
        self.gyro_z = 0.0
        self.imu_monotonic: Optional[float] = None
        self.cmd_monotonic: Optional[float] = None
        self.heading_reference: Optional[float] = None
        self.in_straight_mode = False
        self.last_mode = ''
        self.watchdog_stopped = False

        self.get_logger().info(
            f'Correcteur de cap prêt : profil={self.profile}, '
            f'Kcap={self.kcap:.3f}, Kgyro={self.kgyro:.3f}, '
            f'biais={self.bias:.3f} rad/s, entrée={self.input_topic}, '
            f'sortie={self.output_topic}'
        )

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.gyro_z = float(msg.angular_velocity.z)
        self.imu_monotonic = time.monotonic()

    def imu_fresh(self, now: float) -> bool:
        return (
            self.yaw is not None
            and self.imu_monotonic is not None
            and now - self.imu_monotonic <= self.imu_timeout
        )

    def publish_mode(self, mode: str) -> None:
        if mode == self.last_mode:
            return
        self.last_mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Mode commande : {mode}')

    def publish_stop(self, reason: str) -> None:
        self.heading_reference = None
        self.in_straight_mode = False
        self.cmd_pub.publish(Twist())
        self.publish_mode(reason)

    def cmd_callback(self, msg: Twist) -> None:
        now = time.monotonic()
        self.cmd_monotonic = now
        self.watchdog_stopped = False

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        if abs(linear) < 1.0e-4 and abs(angular) < 1.0e-4:
            self.publish_stop('STOP')
            return

        # Une IMU absente ou figée entraîne un arrêt, y compris en rotation.
        if not self.imu_fresh(now):
            self.publish_stop('STOP_IMU_STALE')
            return

        forward_ok = linear >= self.min_linear
        if not self.apply_forward_only:
            forward_ok = abs(linear) >= self.min_linear

        abs_angular = abs(angular)
        if self.in_straight_mode:
            if not forward_ok or abs_angular >= self.exit_angular:
                self.in_straight_mode = False
                self.heading_reference = None
        elif forward_ok and abs_angular <= self.enter_angular:
            self.in_straight_mode = True
            self.heading_reference = self.yaw

        output = Twist()
        output.linear.x = linear
        output.linear.y = float(msg.linear.y)
        output.linear.z = float(msg.linear.z)
        output.angular.x = float(msg.angular.x)
        output.angular.y = float(msg.angular.y)

        if self.in_straight_mode and self.heading_reference is not None:
            error = wrap_angle(self.heading_reference - float(self.yaw))
            error = clamp(error, -self.max_heading_error, self.max_heading_error)
            correction = self.bias + self.kcap * error - self.kgyro * self.gyro_z
            correction = clamp(correction, -self.max_correction, self.max_correction)
            output.angular.z = clamp(
                angular + correction,
                -self.max_angular,
                self.max_angular,
            )
            self.publish_mode('STRAIGHT_CORRECTED')
        else:
            # Aucun biais en courbe, rotation sur place ou marche arrière.
            output.angular.z = clamp(angular, -self.max_angular, self.max_angular)
            if abs(linear) < self.min_linear and abs_angular > self.enter_angular:
                self.publish_mode('ROTATION_PASSTHROUGH')
            elif linear < 0.0:
                self.publish_mode('REVERSE_PASSTHROUGH')
            else:
                self.publish_mode('CURVE_PASSTHROUGH')

        self.cmd_pub.publish(output)

    def watchdog_callback(self) -> None:
        now = time.monotonic()
        command_stale = (
            self.cmd_monotonic is not None
            and now - self.cmd_monotonic > self.cmd_timeout
        )
        if command_stale and not self.watchdog_stopped:
            self.watchdog_stopped = True
            self.publish_stop('STOP_CMD_TIMEOUT')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
