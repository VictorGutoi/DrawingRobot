import random
from math import isclose, pi

from drawingrobot.kinematics import Pose, step
from drawingrobot.odometry import update_from_encoders


def test_straight_line():
    pose = update_from_encoders(Pose(0.0, 0.0, 0.0), 10.0, 10.0, wheelbase_cm=20.0)
    assert isclose(pose.x, 10.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)
    assert isclose(pose.theta, 0.0, abs_tol=1e-9)


def test_in_place_rotation_no_translation():
    # Equal-magnitude opposite deltas — rotation in place, no XY motion.
    L = 20.0
    pose = update_from_encoders(Pose(0.0, 0.0, 0.0), -5.0, 5.0, wheelbase_cm=L)
    assert isclose(pose.x, 0.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)
    assert isclose(pose.theta, (5.0 - (-5.0)) / L, abs_tol=1e-9)


def test_half_circle_arc():
    # Half circle of radius 10 starting at origin facing +x:
    # arc length per wheel: outer = pi * (R + L/2), inner = pi * (R - L/2)
    R = 10.0
    L = 20.0
    dL = pi * (R - L / 2)   # inner (left)
    dR = pi * (R + L / 2)   # outer (right)
    pose = update_from_encoders(Pose(0.0, 0.0, 0.0), dL, dR, wheelbase_cm=L)
    # End at (0, 2R, pi).
    assert isclose(pose.x, 0.0, abs_tol=1e-9)
    assert isclose(pose.y, 2 * R, abs_tol=1e-9)
    assert isclose(pose.theta, pi, abs_tol=1e-9)


def test_equivalence_with_kinematics_step():
    rng = random.Random(0xDA)
    L = 20.0
    for _ in range(50):
        x = rng.uniform(-30, 30)
        y = rng.uniform(-30, 30)
        th = rng.uniform(-pi, pi)
        v_L = rng.uniform(-20, 20)
        v_R = rng.uniform(-20, 20)
        dt = rng.uniform(0.01, 0.5)
        pose0 = Pose(x, y, th)
        # update_from_encoders(pose, vL*dt, vR*dt, L) ≡ step(pose, vL, vR, L, dt)
        ours = update_from_encoders(pose0, v_L * dt, v_R * dt, L)
        ref = step(pose0, v_L, v_R, L, dt)
        assert isclose(ours.x, ref.x, abs_tol=1e-9)
        assert isclose(ours.y, ref.y, abs_tol=1e-9)
        assert isclose(ours.theta, ref.theta, abs_tol=1e-9)
