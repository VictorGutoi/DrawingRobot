from dataclasses import dataclass
from math import cos, sin

EPSILON = 1e-9


@dataclass
class Pose:
    x: float
    y: float
    theta: float


def step(pose: Pose, v_left: float, v_right: float, wheelbase: float, dt: float) -> Pose:
    """Advance pose by one timestep under differential-drive kinematics.

    Pose is the wheel-axis midpoint. Frame: x forward, y left, theta CCW from +x.
    Exact arc integration (not Euler), so a single step over a full circle is exact.
    """
    if wheelbase <= 0:
        raise ValueError("wheelbase must be positive")

    v = 0.5 * (v_left + v_right)
    omega = (v_right - v_left) / wheelbase

    if abs(omega) < EPSILON:
        return Pose(
            x=pose.x + v * cos(pose.theta) * dt,
            y=pose.y + v * sin(pose.theta) * dt,
            theta=pose.theta,
        )

    theta_new = pose.theta + omega * dt
    R = v / omega
    return Pose(
        x=pose.x - R * sin(pose.theta) + R * sin(theta_new),
        y=pose.y + R * cos(pose.theta) - R * cos(theta_new),
        theta=theta_new,
    )


def transform_point(pose: Pose, body_x: float, body_y: float) -> tuple[float, float]:
    c, s = cos(pose.theta), sin(pose.theta)
    return pose.x + body_x * c - body_y * s, pose.y + body_x * s + body_y * c
