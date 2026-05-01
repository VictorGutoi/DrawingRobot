from math import hypot, isclose, pi

import pytest

from drawingrobot.commands import (
    CommandRunner,
    arc,
    circle,
    move_straight,
    rotate_in_place,
)
from drawingrobot.kinematics import Pose, step


def integrate(commands, wheelbase: float, dt: float = 0.001) -> Pose:
    pose = Pose(0.0, 0.0, 0.0)
    runner = CommandRunner(commands)
    while not runner.done:
        for v_l, v_r, sub_dt in runner.consume(dt):
            pose = step(pose, v_l, v_r, wheelbase, sub_dt)
    return pose


def test_move_straight_distance():
    cmd = move_straight(10.0, speed=2.0)
    assert isclose(cmd.duration, 5.0)
    assert cmd.v_left == cmd.v_right == 2.0


def test_move_straight_negative():
    cmd = move_straight(-4.0, speed=2.0)
    assert isclose(cmd.duration, 2.0)
    assert cmd.v_left == cmd.v_right == -2.0


def test_rotate_in_place_signs():
    cmd = rotate_in_place(pi, wheelbase=1.0, angular_speed=1.0)
    assert isclose(cmd.duration, pi)
    assert cmd.v_left < 0 < cmd.v_right


def test_arc_left_turn_outer_wheel_faster():
    cmd = arc(radius=5.0, angle=pi / 2, wheelbase=1.0, speed=2.0)
    assert cmd.v_right > cmd.v_left


def test_arc_right_turn_inner_wheel_slower():
    cmd = arc(radius=5.0, angle=-pi / 2, wheelbase=1.0, speed=2.0)
    assert cmd.v_left > cmd.v_right


def test_full_circle_returns_close_to_start():
    cmd = circle(radius=3.0, wheelbase=1.0, speed=1.0)
    pose = integrate([cmd], wheelbase=1.0, dt=0.001)
    assert hypot(pose.x, pose.y) < 1e-3
    assert isclose(pose.theta % (2 * pi), 0.0, abs_tol=1e-3) or \
        isclose(pose.theta % (2 * pi), 2 * pi, abs_tol=1e-3)


def test_runner_advances_through_sequence():
    cmds = [move_straight(5.0, speed=1.0), move_straight(5.0, speed=1.0)]
    runner = CommandRunner(cmds)
    assert runner.current_velocities() == (1.0, 1.0)
    runner.advance(5.0)
    assert not runner.done
    runner.advance(5.0)
    assert runner.done
    assert runner.current_velocities() == (0.0, 0.0)


def test_runner_handles_overshoot():
    cmds = [move_straight(1.0, speed=1.0), move_straight(1.0, speed=1.0)]
    runner = CommandRunner(cmds)
    runner.advance(2.5)
    assert runner.done


def test_invalid_inputs():
    with pytest.raises(ValueError):
        move_straight(1.0, speed=0.0)
    with pytest.raises(ValueError):
        rotate_in_place(1.0, wheelbase=1.0, angular_speed=0.0)
    with pytest.raises(ValueError):
        arc(radius=0.0, angle=1.0, wheelbase=1.0, speed=1.0)
