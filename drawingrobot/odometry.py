"""Wheel-encoder odometry update.

Same exact-arc math as `kinematics.step`. step takes (v_left, v_right, dt)
and integrates over the timestep; here we take per-wheel distance deltas
(Δd_L, Δd_R) measured by encoders between two samples. Setting dt=1 and
feeding the deltas as velocities makes the integrals identical:
    v·dt = (Δd/1)·1 = Δd
So we delegate to `step` to guarantee bit-identical math with the rest of
the simulator — the encoder-pose and intended-pose families share one
integrator and can't drift relative to each other for numerical reasons.

Units: deltas and wheelbase in cm (matching the rest of the sim). The
ROS-side SensorListener is responsible for converting m → cm at the wire.
"""

from .kinematics import Pose, step


def update_from_encoders(pose: Pose, d_left_delta_cm: float,
                         d_right_delta_cm: float,
                         wheelbase_cm: float) -> Pose:
    return step(pose, d_left_delta_cm, d_right_delta_cm, wheelbase_cm, 1.0)
