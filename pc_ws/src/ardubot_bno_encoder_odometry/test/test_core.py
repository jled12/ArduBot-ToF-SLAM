import math

from ardubot_bno_encoder_odometry.core import (
    correction_frame_to_raw_odom,
    integrate_encoder_step,
    recenter_heading,
    update_progressive_heading,
    wrap_angle,
)


def test_straight_integrates_x():
    pose = integrate_encoder_step(0, 0, 0, 0.5, 1, 0, 0, True)
    assert abs(pose.x - 0.5) < 1e-12
    assert abs(pose.y) < 1e-12


def test_rotation_freezes_xy():
    pose = integrate_encoder_step(1, 2, 3, 0.15, 0, 0, math.pi/2, False)
    assert (pose.x, pose.y, pose.path_length) == (1, 2, 3)


def test_reverse_is_negative():
    pose = integrate_encoder_step(0, 0, 0, 0.25, -1, 0, 0, True)
    assert abs(pose.x + 0.25) < 1e-12


def test_curve_uses_mid_heading():
    pose = integrate_encoder_step(0, 0, 0, 1, 1, 0, math.pi/2, True)
    assert abs(pose.x - math.sqrt(0.5)) < 1e-12
    assert abs(pose.y - math.sqrt(0.5)) < 1e-12


def test_heading_initializes_from_quaternion():
    out = update_progressive_heading(None, 0.3, 0.0, 0.1, 8.0, 0.12)
    assert abs(out.fused_unwrapped - 0.3) < 1e-12


def test_heading_integrates_gyro():
    out = update_progressive_heading(0.0, 0.0, 1.0, 0.1, 8.0, 0.0)
    assert abs(out.fused_unwrapped - 0.1) < 1e-12


def test_recenter_is_limited():
    fused, applied = recenter_heading(0.0, math.pi/2, math.radians(15))
    assert abs(applied - math.radians(15)) < 1e-12
    assert abs(fused - applied) < 1e-12


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-12


def test_correction_transform_composes_to_desired_pose():
    desired_x, desired_y, desired_yaw = 1.2, -0.4, math.radians(35.0)
    raw_x, raw_y, raw_yaw = 0.7, 0.2, math.radians(-20.0)
    corr = correction_frame_to_raw_odom(
        desired_x, desired_y, desired_yaw, raw_x, raw_y, raw_yaw
    )
    c = math.cos(corr.yaw)
    s = math.sin(corr.yaw)
    composed_x = corr.x + c * raw_x - s * raw_y
    composed_y = corr.y + s * raw_x + c * raw_y
    composed_yaw = wrap_angle(corr.yaw + raw_yaw)
    assert abs(composed_x - desired_x) < 1e-12
    assert abs(composed_y - desired_y) < 1e-12
    assert abs(wrap_angle(composed_yaw - desired_yaw)) < 1e-12


def test_correction_is_identity_when_raw_equals_desired():
    corr = correction_frame_to_raw_odom(0.4, -0.2, 0.3, 0.4, -0.2, 0.3)
    assert abs(corr.x) < 1e-12
    assert abs(corr.y) < 1e-12
    assert abs(corr.yaw) < 1e-12
