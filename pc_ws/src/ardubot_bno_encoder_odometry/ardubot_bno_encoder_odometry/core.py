from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass(frozen=True)
class HeadingUpdate:
    fused_unwrapped: float
    predicted_unwrapped: float
    innovation: float
    quaternion_weight: float


def update_progressive_heading(
    previous_unwrapped: float | None,
    quaternion_yaw: float,
    gyro_z: float,
    dt: float,
    quaternion_time_constant: float,
    quaternion_weight_scale: float,
) -> HeadingUpdate:
    if previous_unwrapped is None:
        return HeadingUpdate(quaternion_yaw, quaternion_yaw, 0.0, 1.0)

    dt = clamp(float(dt), 0.0, 0.5)
    predicted = previous_unwrapped + float(gyro_z) * dt
    predicted_wrapped = wrap_angle(predicted)
    innovation = wrap_angle(float(quaternion_yaw) - predicted_wrapped)
    tau = max(float(quaternion_time_constant), 1.0e-3)
    base_weight = 1.0 - math.exp(-dt / tau)
    weight = clamp(base_weight * float(quaternion_weight_scale), 0.0, 1.0)
    fused = predicted + weight * innovation
    return HeadingUpdate(fused, predicted, innovation, weight)


def recenter_heading(fused_unwrapped: float, quaternion_yaw: float, max_jump_rad: float) -> tuple[float, float]:
    requested = wrap_angle(float(quaternion_yaw) - wrap_angle(float(fused_unwrapped)))
    applied = clamp(requested, -abs(float(max_jump_rad)), abs(float(max_jump_rad)))
    return fused_unwrapped + applied, applied


@dataclass(frozen=True)
class IntegratedPose:
    x: float
    y: float
    path_length: float


def integrate_encoder_step(
    x: float,
    y: float,
    path_length: float,
    ds_unsigned: float,
    direction: float,
    previous_yaw_unwrapped: float,
    current_yaw_unwrapped: float,
    translate: bool,
) -> IntegratedPose:
    if not translate or abs(direction) < 1.0e-9:
        return IntegratedPose(float(x), float(y), float(path_length))
    ds = abs(float(ds_unsigned)) * (1.0 if direction > 0.0 else -1.0)
    theta_mid = 0.5 * (float(previous_yaw_unwrapped) + float(current_yaw_unwrapped))
    return IntegratedPose(
        float(x) + ds * math.cos(theta_mid),
        float(y) + ds * math.sin(theta_mid),
        float(path_length) + abs(ds),
    )


@dataclass(frozen=True)
class CorrectionTransform2D:
    x: float
    y: float
    yaw: float


def correction_frame_to_raw_odom(
    desired_x: float,
    desired_y: float,
    desired_yaw: float,
    raw_x: float,
    raw_y: float,
    raw_yaw: float,
) -> CorrectionTransform2D:
    """Return T_hybrid_raw so T_hybrid_raw * T_raw_base = T_hybrid_base."""
    yaw = wrap_angle(float(desired_yaw) - float(raw_yaw))
    c = math.cos(yaw)
    si = math.sin(yaw)
    rotated_raw_x = c * float(raw_x) - si * float(raw_y)
    rotated_raw_y = si * float(raw_x) + c * float(raw_y)
    return CorrectionTransform2D(
        float(desired_x) - rotated_raw_x,
        float(desired_y) - rotated_raw_y,
        yaw,
    )
