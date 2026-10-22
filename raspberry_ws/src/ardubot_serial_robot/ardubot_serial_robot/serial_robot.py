#!/usr/bin/env python3
"""Pont serie ArduBot -> ROS 2 avec contrat d'unites explicite.

Trames serie attendues :
- CMD_VEL,left,right : m/s par roue
- ENC,left,right : ticks cumules
- IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az : quaternion sans unite, gyro rad/s, acceleration m/s^2
- MAG,mx,my,mz : microteslas par defaut (convertis en teslas pour ROS)
- CAL,sys,gyro,accel,mag : niveaux 0..3

Repere publie : x vers l'avant, y vers la gauche, z vers le haut.
"""
import math
import serial
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import UInt8MultiArray, Int64
from tf2_ros import TransformBroadcaster


class SerialRobotNode(Node):
    def __init__(self):
        super().__init__('serial_robot')

        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('wheel_diameter', 0.030)
        self.declare_parameter('wheel_base', 0.0784)
        self.declare_parameter('ticks_per_wheel_rev', 348.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('read_rate', 100.0)
        self.declare_parameter('cmd_rate', 20.0)
        self.declare_parameter('tf_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_wheel_speed', 0.5)

        self.declare_parameter('odom_xy_covariance', 0.02)
        self.declare_parameter('odom_yaw_covariance', 0.05)
        self.declare_parameter('imu_orientation_covariance', 0.05)
        self.declare_parameter('imu_angular_velocity_covariance', 0.02)
        self.declare_parameter('imu_linear_acceleration_covariance', 0.20)
        self.declare_parameter('magnetic_field_covariance', 0.0)
        self.declare_parameter('mag_input_unit', 'uT')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_radius = self.wheel_diameter / 2.0
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.ticks_per_wheel_rev = float(self.get_parameter('ticks_per_wheel_rev').value)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.read_rate = float(self.get_parameter('read_rate').value)
        self.cmd_rate = float(self.get_parameter('cmd_rate').value)
        self.tf_rate = float(self.get_parameter('tf_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)

        self.odom_xy_covariance = float(self.get_parameter('odom_xy_covariance').value)
        self.odom_yaw_covariance = float(self.get_parameter('odom_yaw_covariance').value)
        self.imu_orientation_covariance = float(self.get_parameter('imu_orientation_covariance').value)
        self.imu_angular_velocity_covariance = float(self.get_parameter('imu_angular_velocity_covariance').value)
        self.imu_linear_acceleration_covariance = float(self.get_parameter('imu_linear_acceleration_covariance').value)
        self.magnetic_field_covariance = float(self.get_parameter('magnetic_field_covariance').value)
        self.mag_input_unit = str(self.get_parameter('mag_input_unit').value).strip().lower()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.left_ticks_pub = self.create_publisher(Int64, '/left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int64, '/right_ticks', 10)
        self.left_ticks_delta_pub = self.create_publisher(Int64, '/left_ticks_delta', 10)
        self.right_ticks_delta_pub = self.create_publisher(Int64, '/right_ticks_delta', 10)
        self.calibration_pub = self.create_publisher(
            UInt8MultiArray, '/imu/calibration', 10
        )
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
            self.get_logger().info(f'Port serie ouvert: {self.port} @ {self.baud}')
            self.get_logger().info(
                'Contrat unites serie: gyro=rad/s, acceleration=m/s^2, '
                f'magnetometre={self.mag_input_unit} -> tesla ROS, CMD_VEL=m/s.'
            )
        except Exception as e:
            self.get_logger().error(f'Impossible ouvrir {self.port}: {e}')
            raise

        self.serial_buffer = ''

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_odom_time = self.get_clock().now()

        self.last_vx = 0.0
        self.last_wz = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.target_left = 0.0
        self.target_right = 0.0

        # Diagnostic UART : conserve exactement les 10 dernières trames
        # CMD_VEL préparées côté Raspberry avant écriture sur /dev/ttyS0.
        self.serial_tx_history = deque(maxlen=10)

        # Avec un seul encodeur par roue, le signe est deduit de la consigne roue.
        self.last_left_sign = 0.0
        self.last_right_sign = 0.0

        self.read_timer = self.create_timer(1.0 / self.read_rate, self.read_serial_timer)
        self.cmd_timer = self.create_timer(1.0 / self.cmd_rate, self.send_cmd_timer)
        self.tf_timer = self.create_timer(1.0 / self.tf_rate, self.publish_tf_loop)

    def clamp(self, value, limit):
        return max(min(value, limit), -limit)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def yaw_to_quat(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def cmd_vel_cb(self, msg):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        left = v - (w * self.wheel_base * 0.5)
        right = v + (w * self.wheel_base * 0.5)

        self.target_left = self.clamp(left, self.max_wheel_speed)
        self.target_right = self.clamp(right, self.max_wheel_speed)

        self.last_left_sign = math.copysign(1.0, self.target_left) if abs(self.target_left) > 1e-4 else 0.0
        self.last_right_sign = math.copysign(1.0, self.target_right) if abs(self.target_right) > 1e-4 else 0.0

        self.last_cmd_time = self.get_clock().now()

    def send_cmd_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        left = self.target_left
        right = self.target_right

        if dt > self.cmd_timeout:
            left = 0.0
            right = 0.0

        line = f'CMD_VEL,{left:.3f},{right:.3f}\n'

        # Mémorise exactement la trame préparée avant son envoi à l'Arduino.
        self.serial_tx_history.append(line.strip())

        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'Erreur ecriture serie: {e}')

    def read_serial_timer(self):
        lines = self.read_serial_lines()
        for line in lines:
            if line.startswith('ENC,'):
                self.handle_enc_line(line)
            elif line.startswith('IMU,'):
                self.handle_imu_line(line)
            elif line.startswith('CAL,'):
                self.handle_calibration_line(line)
            elif line.startswith('MAG,'):
                self.handle_mag_line(line)
            elif line.startswith('INFO,') or line.startswith('WARN,'):
                self.get_logger().info(line)

                # Si l'Arduino détecte une CMD_VEL aberrante, afficher les
                # 10 dernières trames réellement préparées côté Raspberry.
                # CLAMPED = firmware actuel ; REJECTED = firmware sécurisé futur.
                if (line.startswith('WARN,CMD_VEL_CLAMPED') or
                        line.startswith('WARN,CMD_VEL_REJECTED')):
                    self.get_logger().error(
                        'ARDUINO A RECU UNE CMD_VEL ABERRANTE'
                    )
                    for i, tx in enumerate(self.serial_tx_history):
                        self.get_logger().error(
                            f'TX_SERIAL_HISTORY[{i}] = {tx}'
                        )

    def read_serial_lines(self):
        lines = []
        try:
            data = self.ser.read(4096)
            if data:
                self.serial_buffer += data.decode('ascii', errors='ignore')
                while '\n' in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        lines.append(line)
        except Exception as e:
            self.get_logger().error(f'Erreur lecture serie: {e}')
        return lines

    def handle_enc_line(self, line):
        parts = line.split(',')
        if len(parts) != 3:
            return

        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])

            left_msg = Int64()
            left_msg.data = left_ticks
            self.left_ticks_pub.publish(left_msg)

            right_msg = Int64()
            right_msg.data = right_ticks
            self.right_ticks_pub.publish(right_msg)
        except ValueError:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            self.prev_odom_time = now
            return

        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now
            return

        dleft_ticks = left_ticks - self.prev_left_ticks
        dright_ticks = right_ticks - self.prev_right_ticks

        left_delta_msg = Int64()
        left_delta_msg.data = int(dleft_ticks)
        self.left_ticks_delta_pub.publish(left_delta_msg)

        right_delta_msg = Int64()
        right_delta_msg.data = int(dright_ticks)
        self.right_ticks_delta_pub.publish(right_delta_msg)

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_odom_time = now

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_wheel_rev

        dleft = dleft_ticks * meters_per_tick * self.last_left_sign
        dright = dright_ticks * meters_per_tick * self.last_right_sign

        ds = (dleft + dright) / 2.0
        dyaw = (dright - dleft) / self.wheel_base

        self.x += ds * math.cos(self.yaw + dyaw / 2.0)
        self.y += ds * math.sin(self.yaw + dyaw / 2.0)
        self.yaw = self.normalize_angle(self.yaw + dyaw)

        self.last_vx = ds / dt
        self.last_wz = dyaw / dt

        # La pose est mise a jour ici, mais /odom est publie par le timer TF
        # afin que /odom et odom->base_link aient exactement le meme timestamp.

    def publish_odom(self, stamp):
        qz, qw = self.yaw_to_quat(self.yaw)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = self.last_vx
        msg.twist.twist.angular.z = self.last_wz

        msg.pose.covariance[0] = self.odom_xy_covariance
        msg.pose.covariance[7] = self.odom_xy_covariance
        msg.pose.covariance[35] = self.odom_yaw_covariance

        msg.twist.covariance[0] = self.odom_xy_covariance
        msg.twist.covariance[35] = self.odom_yaw_covariance

        self.odom_pub.publish(msg)

    def magnetic_value_to_tesla(self, value):
        """Convertit la valeur série du magnétomètre vers l'unité ROS : tesla."""
        if self.mag_input_unit in ('ut', 'µt', 'microtesla', 'microteslas'):
            return value * 1e-6
        if self.mag_input_unit in ('mt', 'millitesla', 'milliteslas'):
            return value * 1e-3
        if self.mag_input_unit in ('gauss', 'g'):
            return value * 1e-4
        if self.mag_input_unit in ('t', 'tesla', 'teslas'):
            return value

        self.get_logger().warning(
            f"Unité magnétomètre inconnue '{self.mag_input_unit}', valeurs supposées en µT.",
            once=True
        )
        return value * 1e-6

    def publish_magnetic_field(self, mx, my, mz, stamp=None):
        msg = MagneticField()
        msg.header.stamp = (stamp or self.get_clock().now()).to_msg()
        msg.header.frame_id = self.imu_frame

        # sensor_msgs/MagneticField exige des teslas.
        msg.magnetic_field.x = self.magnetic_value_to_tesla(mx)
        msg.magnetic_field.y = self.magnetic_value_to_tesla(my)
        msg.magnetic_field.z = self.magnetic_value_to_tesla(mz)

        c = self.magnetic_field_covariance
        msg.magnetic_field_covariance = [
            c, 0.0, 0.0,
            0.0, c, 0.0,
            0.0, 0.0, c
        ]
        self.mag_pub.publish(msg)

    def handle_mag_line(self, line):
        # Format recommandé Arduino : MAG,mx,my,mz
        parts = line.split(',')
        if len(parts) != 4:
            self.get_logger().warning(
                'Ligne MAG invalide. Format attendu : MAG,mx,my,mz',
                once=True
            )
            return

        try:
            mx = float(parts[1])
            my = float(parts[2])
            mz = float(parts[3])
        except ValueError:
            return

        self.publish_magnetic_field(mx, my, mz)

    def handle_imu_line(self, line):
        # Formats Arduino acceptés :
        # IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
        # q: sans unite; g: rad/s; a: m/s^2 (acceleration lineaire, gravite retiree)
        # IMU,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz
        parts = line.split(',')
        if len(parts) not in (11, 14):
            self.get_logger().warning(
                'Ligne IMU invalide. 11 champs attendus, ou 14 avec mx,my,mz.',
                once=True
            )
            return

        try:
            qw = float(parts[1])
            qx = float(parts[2])
            qy = float(parts[3])
            qz = float(parts[4])
            gx = float(parts[5])
            gy = float(parts[6])
            gz = float(parts[7])
            ax = float(parts[8])
            ay = float(parts[9])
            az = float(parts[10])
            if len(parts) == 14:
                mx = float(parts[11])
                my = float(parts[12])
                mz = float(parts[13])
            else:
                mx = my = mz = None
        except ValueError:
            return

        now = self.get_clock().now()
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.imu_frame

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz

        # sensor_msgs/Imu exige des rad/s. Le firmware Arduino convertit deja deg/s -> rad/s.
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # sensor_msgs/Imu exige des m/s^2. VECTOR_LINEARACCEL est deja en m/s^2.
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation_covariance = [
            self.imu_orientation_covariance, 0.0, 0.0,
            0.0, self.imu_orientation_covariance, 0.0,
            0.0, 0.0, self.imu_orientation_covariance
        ]
        msg.angular_velocity_covariance = [
            self.imu_angular_velocity_covariance, 0.0, 0.0,
            0.0, self.imu_angular_velocity_covariance, 0.0,
            0.0, 0.0, self.imu_angular_velocity_covariance
        ]
        msg.linear_acceleration_covariance = [
            self.imu_linear_acceleration_covariance, 0.0, 0.0,
            0.0, self.imu_linear_acceleration_covariance, 0.0,
            0.0, 0.0, self.imu_linear_acceleration_covariance
        ]

        self.imu_pub.publish(msg)

        if mx is not None:
            self.publish_magnetic_field(mx, my, mz, stamp=now)


    def handle_calibration_line(self, line):
        # CAL,systeme,gyroscope,accelerometre,magnetometre
        parts = line.split(',')
        if len(parts) != 5:
            return

        try:
            values = [int(value) for value in parts[1:5]]
        except ValueError:
            return

        if any(value < 0 or value > 3 for value in values):
            return

        msg = UInt8MultiArray()
        msg.data = values
        self.calibration_pub.publish(msg)

    def publish_tf_loop(self):
        now = self.get_clock().now()

        # Publier /odom au meme rythme que le TF (tf_rate) et avec exactement
        # le meme timestamp. Cela evite que /odom reste date du dernier ENC
        # alors que odom->base_link continue d'etre publie avec l'heure courante.
        self.publish_odom(now)

        if not self.publish_tf:
            return

        qz, qw = self.yaw_to_quat(self.yaw)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(b'CMD_VEL,0.000,0.000\n')
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialRobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
