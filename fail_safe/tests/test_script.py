"""Tests for the fail-safe script DSL and command builders.

The load-bearing test is `test_turn_keeps_pen_still`: it verifies the whole
point of the fail-safe — pivoting the right wheel around a locked left
wheel leaves the pen (at the left wheel) exactly where it started.
"""

from math import isclose, pi, radians

import pytest

from drawingrobot_failsafe.kinematics import Pose, step, transform_point
from drawingrobot_failsafe.script import (
    CommandRunner,
    DEFAULT_ANGULAR_SPEED_DEG,
    DEFAULT_SPEED,
    ScriptError,
    WheelCommand,
    move_straight,
    parse_script,
    pen_circle,
    pivot_turn,
    rescale_runner,
)


WHEELBASE = 20.4  # real-robot width, cm
PEN_BODY = (0.0, WHEELBASE / 2)   # left wheel position, body frame


def _integrate(cmds, wheelbase=WHEELBASE, dt=1.0 / 240.0):
    """Step a runner to completion at fine dt, return final pose + pen path."""
    runner = CommandRunner(cmds)
    pose = Pose(0.0, 0.0, 0.0)
    pen_world: list[tuple[float, float]] = [transform_point(pose, *PEN_BODY)]
    while not runner.done:
        for v_l, v_r, sub_dt in runner.consume(dt):
            pose = step(pose, v_l, v_r, wheelbase, sub_dt)
        pen_world.append(transform_point(pose, *PEN_BODY))
    return pose, pen_world


# ---------- builders ----------

def test_move_straight_distance_matches():
    cmd = move_straight(20.0, speed=10.0)
    assert isclose(cmd.duration, 2.0)
    assert cmd.v_left == cmd.v_right == 10.0


def test_move_straight_negative_reverses():
    cmd = move_straight(-5.0, speed=10.0)
    assert cmd.v_left == cmd.v_right == -10.0
    assert isclose(cmd.duration, 0.5)


def test_pivot_turn_keeps_left_wheel_v_zero():
    cmd = pivot_turn(radians(90), wheelbase=20.0, angular_speed=pi)
    assert cmd.v_left == 0.0
    # v_right = +π · 20 = 20π for CCW (positive angle)
    assert isclose(cmd.v_right, 20 * pi)


def test_pivot_turn_negative_reverses_right_wheel():
    cmd = pivot_turn(radians(-90), wheelbase=20.0, angular_speed=pi)
    assert cmd.v_left == 0.0
    assert isclose(cmd.v_right, -20 * pi)


def test_pen_circle_velocity_ratios():
    cmd = pen_circle(radius_cm=10.0, wheelbase=20.0, pen_speed=12.0)
    # ω = 12/10 = 1.2; v_left = 12, v_right = 1.2 * 30 = 36
    assert isclose(cmd.v_left, 12.0)
    assert isclose(cmd.v_right, 36.0)
    # Full circle in 2π/ω = 2π/1.2 ≈ 5.236 s
    assert isclose(cmd.duration, 2 * pi / 1.2)


# ---------- the load-bearing invariant ----------

def test_turn_keeps_pen_still():
    """Pen (at left wheel) must NOT move during a `turn`."""
    cmds = [pivot_turn(radians(90), WHEELBASE, radians(DEFAULT_ANGULAR_SPEED_DEG))]
    _, pen_world = _integrate(cmds)
    # Start and end pen positions identical to within numerical noise.
    assert all(isclose(p[0], pen_world[0][0], abs_tol=1e-9) for p in pen_world)
    assert all(isclose(p[1], pen_world[0][1], abs_tol=1e-9) for p in pen_world)


def test_turn_rotates_robot_around_pen():
    """After a 90° turn, the robot heading is +π/2 and pen position unchanged."""
    cmds = [pivot_turn(radians(90), WHEELBASE, radians(DEFAULT_ANGULAR_SPEED_DEG))]
    pose, _ = _integrate(cmds)
    assert isclose(pose.theta, pi / 2, abs_tol=1e-6)
    # Wheel-midpoint must have moved (it sweeps an arc of radius W/2 around
    # the left wheel). Specifically, it ends up at world (W/2, +W/2).
    assert isclose(pose.x, WHEELBASE / 2, abs_tol=1e-6)
    assert isclose(pose.y, WHEELBASE / 2, abs_tol=1e-6)


def test_square_closes():
    """Four (forward 20, turn 90) pairs return to start with sharp corners."""
    cmds: list[WheelCommand] = []
    for _ in range(4):
        cmds.append(move_straight(20.0, DEFAULT_SPEED))
        cmds.append(pivot_turn(radians(90), WHEELBASE,
                               radians(DEFAULT_ANGULAR_SPEED_DEG)))
    pose, pen_world = _integrate(cmds)
    # Pen returns to start (within float noise from integration).
    start = pen_world[0]
    end = pen_world[-1]
    assert isclose(end[0], start[0], abs_tol=1e-4)
    assert isclose(end[1], start[1], abs_tol=1e-4)


def test_circle_closes():
    """One full `pen_circle` returns the pen to its starting position."""
    cmds = [pen_circle(15.0, WHEELBASE, pen_speed=12.0)]
    pose, pen_world = _integrate(cmds, dt=1.0 / 480.0)
    start = pen_world[0]
    end = pen_world[-1]
    assert isclose(end[0], start[0], abs_tol=1e-3)
    assert isclose(end[1], start[1], abs_tol=1e-3)


# ---------- parser ----------

def test_parse_forward_turn_circle():
    src = """
    # comment line
    speed 12
    forward 20
    turn 90
    circle 15
    """
    cmds = parse_script(src, wheelbase=WHEELBASE)
    assert len(cmds) == 3
    # forward 20 at speed 12 → duration 20/12
    assert isclose(cmds[0].duration, 20 / 12)
    # turn 90 at default angular_speed=π rad/s → duration (π/2)/π = 0.5
    assert isclose(cmds[1].duration, 0.5, abs_tol=1e-9)
    # circle 15 at pen_speed=12 → duration 2π·15/12
    assert isclose(cmds[2].duration, 2 * pi * 15 / 12)


def test_parse_rejects_unknown_command():
    with pytest.raises(ScriptError) as exc_info:
        parse_script("trace 0 0 10 10", wheelbase=WHEELBASE)
    assert exc_info.value.line_no == 1
    assert "unknown command" in str(exc_info.value)


def test_parse_rejects_zero_speed():
    with pytest.raises(ScriptError):
        parse_script("speed 0", wheelbase=WHEELBASE)


def test_parse_rejects_negative_circle_radius():
    with pytest.raises(ScriptError):
        parse_script("circle -3", wheelbase=WHEELBASE)


# ---------- runner ----------

def test_command_runner_consume_splits_at_boundaries():
    cmds = [
        WheelCommand(v_left=1.0, v_right=1.0, duration=1.0),
        WheelCommand(v_left=2.0, v_right=2.0, duration=1.0),
    ]
    r = CommandRunner(cmds)
    segs = r.consume(1.5)
    # 1.0 of cmd0, then 0.5 of cmd1
    assert segs == [(1.0, 1.0, 1.0), (2.0, 2.0, 0.5)]


def test_rescale_runner_preserves_total_distance():
    cmds = [WheelCommand(10.0, 10.0, 2.0)]
    r = CommandRunner(cmds)
    scaled = rescale_runner(r, target_time=4.0)
    # Velocities halve, duration doubles.
    c = scaled._commands[0]
    assert isclose(c.v_left, 5.0)
    assert isclose(c.duration, 4.0)


# ---------- inject (encoder-feedback corrections) ----------

def test_inject_empty_is_noop():
    r = CommandRunner([WheelCommand(1.0, 1.0, 1.0), WheelCommand(2.0, 2.0, 1.0)])
    r.inject([])
    assert len(r._commands) == 2


def test_inject_mid_run_splices_after_current():
    a = WheelCommand(1.0, 1.0, 2.0)
    b = WheelCommand(2.0, 2.0, 3.0)
    r = CommandRunner([a, b])
    r.advance(1.0)            # mid-way through a
    assert r._idx == 0
    corr = WheelCommand(5.0, 5.0, 0.5)
    r.inject([corr])
    assert r._commands[0] is a
    assert r._commands[1] is corr
    assert r._commands[2] is b
    assert r._elapsed == 1.0  # current command's progress preserved


def test_inject_when_done_revives_runner():
    a = WheelCommand(1.0, 1.0, 1.0)
    r = CommandRunner([a])
    r.advance(1.5)
    assert r.done
    r.inject([WheelCommand(3.0, 3.0, 0.5)])
    assert not r.done
    assert r.current_velocities() == (3.0, 3.0)
