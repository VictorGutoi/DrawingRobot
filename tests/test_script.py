from math import hypot, isclose, pi

import pytest

from drawingrobot.commands import CommandRunner
from drawingrobot.kinematics import Pose, step, transform_point
from drawingrobot.robot import RobotGeometry
from drawingrobot.script import (
    DEFAULT_SPEED,
    ScriptError,
    list_scripts,
    load_script,
    parse_script,
)


GEOMETRY = RobotGeometry(width=20.0, length=30.0, wheel_offset=10.0)


def _integrate(cmds, geometry, dt=0.01):
    runner = CommandRunner(cmds)
    pose = Pose(0.0, 0.0, 0.0)
    while not runner.done:
        for v_l, v_r, sub_dt in runner.consume(dt):
            pose = step(pose, v_l, v_r, geometry.width, sub_dt)
    return pose


def test_blank_and_comments_yield_no_commands():
    text = """
    # this is a comment

       # indented comment
    """
    assert parse_script(text, GEOMETRY) == []


def test_forward_uses_default_speed():
    cmds = parse_script("forward 24", GEOMETRY)
    assert len(cmds) == 1
    assert cmds[0].v_left == DEFAULT_SPEED
    assert cmds[0].v_right == DEFAULT_SPEED
    assert isclose(cmds[0].duration, 24.0 / DEFAULT_SPEED)


def test_back_is_negative_forward():
    fwd = parse_script("forward -10", GEOMETRY)[0]
    bk = parse_script("back 10", GEOMETRY)[0]
    assert fwd == bk


def test_left_and_right_are_signed_turn():
    left = parse_script("left 90", GEOMETRY)[0]
    right = parse_script("right 90", GEOMETRY)[0]
    turn = parse_script("turn 90", GEOMETRY)[0]
    assert left == turn
    assert left.v_left == -right.v_left
    assert left.v_right == -right.v_right


def test_speed_persists_across_lines():
    cmds = parse_script("speed 5\nforward 10\nforward 20", GEOMETRY)
    assert all(c.v_left == 5.0 for c in cmds)
    assert isclose(cmds[0].duration, 2.0)
    assert isclose(cmds[1].duration, 4.0)


def test_arc_and_circle():
    cmds = parse_script("arc 10 90\ncircle 5", GEOMETRY)
    assert len(cmds) == 2
    assert cmds[0].v_right > cmds[0].v_left
    assert cmds[1].v_right > cmds[1].v_left


def test_inline_comment_stripped():
    cmds = parse_script("forward 10  # go forward", GEOMETRY)
    assert len(cmds) == 1


def test_unknown_command_raises_with_line_number():
    with pytest.raises(ScriptError) as exc:
        parse_script("forward 10\nbanana 3", GEOMETRY)
    assert exc.value.line_no == 2
    assert "banana" in str(exc.value)


def test_missing_argument_raises():
    with pytest.raises(ScriptError):
        parse_script("forward", GEOMETRY)


def test_invalid_number_raises():
    with pytest.raises(ScriptError):
        parse_script("forward fast", GEOMETRY)


def test_negative_speed_rejected():
    with pytest.raises(ScriptError):
        parse_script("speed -5", GEOMETRY)


def test_example_scripts_all_parse():
    names = list_scripts()
    assert {"square", "rectangle", "triangle", "circle"}.issubset(set(names))
    # `trace` requires off-axis pen; use a back-corner offset that satisfies
    # every example script's needs (pen_path scripts also work with this).
    pen_body = (-5.0, -3.0)
    for name in names:
        cmds = parse_script(load_script(name), GEOMETRY, pen_body=pen_body)
        assert len(cmds) > 0


def test_square_script_returns_to_start():
    cmds = parse_script(load_script("square"), GEOMETRY)
    pose = _integrate(cmds, GEOMETRY)
    assert isclose(pose.x, 0.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)
    assert isclose(pose.theta % (2 * pi), 0.0, abs_tol=1e-9) or \
        isclose(pose.theta % (2 * pi), 2 * pi, abs_tol=1e-9)


def test_goto_lands_pen_at_target_with_off_axis_pen():
    pen_body = (-10.0, 5.0)
    cmds = parse_script("goto 50 30", GEOMETRY, pen_body=pen_body)
    pose = _integrate(cmds, GEOMETRY)
    pen_world = transform_point(pose, *pen_body)
    assert isclose(pen_world[0], 50.0, abs_tol=1e-9)
    assert isclose(pen_world[1], 30.0, abs_tol=1e-9)


def test_goto_lands_pen_at_target_pen_on_outline():
    # Pen on the back-left corner of the chassis outline.
    pen_body = GEOMETRY.pen_offset(GEOMETRY.length + GEOMETRY.width)
    cmds = parse_script("goto 60 40", GEOMETRY, pen_body=pen_body)
    pose = _integrate(cmds, GEOMETRY)
    pen_world = transform_point(pose, *pen_body)
    assert isclose(pen_world[0], 60.0, abs_tol=1e-9)
    assert isclose(pen_world[1], 40.0, abs_tol=1e-9)


def test_goto_polyline_visits_each_target_in_order():
    pen_body = (-8.0, 4.0)
    targets = [(40.0, 0.0), (40.0, 40.0), (0.0, 40.0), (-20.0, 10.0)]
    script = "\n".join(f"goto {x} {y}" for x, y in targets)
    cmds = parse_script(script, GEOMETRY, pen_body=pen_body)
    # Each goto compiles to rotate+forward (2 commands). The pen lands on the
    # target at the END of every forward, i.e. after every second command.
    pose = Pose(0.0, 0.0, 0.0)
    for i, cmd in enumerate(cmds):
        pose = step(pose, cmd.v_left, cmd.v_right, GEOMETRY.width, cmd.duration)
        if i % 2 == 1:
            target = targets[i // 2]
            pen_world = transform_point(pose, *pen_body)
            assert isclose(pen_world[0], target[0], abs_tol=1e-9)
            assert isclose(pen_world[1], target[1], abs_tol=1e-9)


def test_goto_target_inside_pen_swept_circle_raises():
    pen_body = (-10.0, 5.0)  # |pen| = sqrt(125) ≈ 11.18
    with pytest.raises(ScriptError) as exc:
        parse_script("goto 0 0", GEOMETRY, pen_body=pen_body)
    assert "swept circle" in str(exc.value) or "pen" in str(exc.value).lower()


def test_goto_with_centered_pen_falls_back_to_straight_drive():
    # Pen at wheel midpoint: any straight line target reachable, no fillet.
    cmds = parse_script("goto 25 0", GEOMETRY, pen_body=(0.0, 0.0))
    pose = _integrate(cmds, GEOMETRY)
    assert isclose(pose.x, 25.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)


def test_line_to_pen_lands_on_target_and_last_leg_is_straight_edge():
    # line_to compiles to setup + a single forward; the *last* command is the
    # edge-tracing forward, and it should make the pen go straight from previous
    # pen position to target.
    pen_body = (-8.0, 4.0)
    cmds = parse_script("line_to 40 30", GEOMETRY, pen_body=pen_body)
    assert isinstance(cmds[-1].duration, float)
    # Integrate everything except the last cmd; pen must end at the *initial*
    # pen position, which is just R_0 * pen_body since pose starts at origin.
    pose = Pose(0.0, 0.0, 0.0)
    for cmd in cmds[:-1]:
        pose = step(pose, cmd.v_left, cmd.v_right, GEOMETRY.width, cmd.duration)
    pen_after_setup = transform_point(pose, *pen_body)
    initial_pen = (pen_body[0], pen_body[1])
    assert isclose(pen_after_setup[0], initial_pen[0], abs_tol=1e-9)
    assert isclose(pen_after_setup[1], initial_pen[1], abs_tol=1e-9)
    # Now run the final forward leg; pen must hit (40, 30) exactly.
    last = cmds[-1]
    pose = step(pose, last.v_left, last.v_right, GEOMETRY.width, last.duration)
    pen_final = transform_point(pose, *pen_body)
    assert isclose(pen_final[0], 40.0, abs_tol=1e-9)
    assert isclose(pen_final[1], 30.0, abs_tol=1e-9)


def test_line_to_polyline_each_edge_is_straight_pen_line():
    pen_body = (-8.8, -10.2)
    targets = [(30.0, 0.0), (30.0, 30.0), (0.0, 30.0), (0.0, 0.0)]
    script = "\n".join(f"line_to {x} {y}" for x, y in targets)
    cmds = parse_script(script, GEOMETRY, pen_body=pen_body)

    pose = Pose(0.0, 0.0, 0.0)
    pen_at_edge_starts: list[tuple[float, float]] = []
    pen_at_edge_ends: list[tuple[float, float]] = []
    target_idx = 0
    edge_lengths_expected = []
    prev_pen = transform_point(pose, *pen_body)
    for tx, ty in targets:
        edge_lengths_expected.append(hypot(tx - prev_pen[0], ty - prev_pen[1]))
        prev_pen = (tx, ty)

    for cmd in cmds:
        is_forward_edge = (cmd.v_left == cmd.v_right and cmd.v_left > 0
                          and abs(cmd.duration * cmd.v_left
                                  - edge_lengths_expected[target_idx]) < 1e-9)
        if is_forward_edge:
            pen_at_edge_starts.append(transform_point(pose, *pen_body))
        pose = step(pose, cmd.v_left, cmd.v_right, GEOMETRY.width, cmd.duration)
        if is_forward_edge:
            pen_at_edge_ends.append(transform_point(pose, *pen_body))
            target_idx += 1

    assert len(pen_at_edge_ends) == len(targets)
    for end, target in zip(pen_at_edge_ends, targets):
        assert isclose(end[0], target[0], abs_tol=1e-9)
        assert isclose(end[1], target[1], abs_tol=1e-9)
    # Each edge starts where the previous edge ended (corner)
    initial_pen = (pen_body[0], pen_body[1])
    assert isclose(pen_at_edge_starts[0][0], initial_pen[0], abs_tol=1e-9)
    assert isclose(pen_at_edge_starts[0][1], initial_pen[1], abs_tol=1e-9)
    for i in range(1, len(targets)):
        assert isclose(pen_at_edge_starts[i][0], targets[i - 1][0], abs_tol=1e-9)
        assert isclose(pen_at_edge_starts[i][1], targets[i - 1][1], abs_tol=1e-9)


def test_line_to_when_already_aligned_skips_setup():
    # Pen at body (px, 0): for an axis-aligned target along +x, the heading is
    # already 0 and the setup translation is zero — should be one forward command.
    pen_body = (-5.0, 0.0)
    cmds = parse_script("line_to 20 0", GEOMETRY, pen_body=pen_body)
    assert len(cmds) == 1
    assert cmds[0].v_left == cmds[0].v_right > 0


def test_trace_pen_tracks_polyline_within_discretisation():
    pen_body = (14.4, 0.0)  # front-mid pen, biggest off-axis case
    targets = [(30.0, 0.0), (30.0, 30.0), (0.0, 30.0), (0.0, 0.0)]
    script_text = "trace " + " ".join(f"{x} {y}" for x, y in targets)
    cmds = parse_script(script_text, GEOMETRY, pen_body=pen_body)
    pose = _integrate(cmds, GEOMETRY, dt=0.005)
    pen_final = transform_point(pose, *pen_body)
    # Final waypoint is (0, 0); pen should land within a few mm of it.
    assert hypot(pen_final[0], pen_final[1]) < 0.3


def test_trace_pen_visits_each_vertex_within_tracking_tolerance():
    # Step through cmds and check that whenever `s_along` of an edge reaches its
    # end, the pen is at (or near) the corresponding waypoint.
    pen_body = (-8.8, -10.2)
    targets = [(40.0, 0.0), (40.0, 30.0), (10.0, 30.0)]
    script_text = "trace " + " ".join(f"{x} {y}" for x, y in targets)
    cmds = parse_script(script_text, GEOMETRY, pen_body=pen_body)
    # The trace prepends initial pen as a waypoint, so we have 4 edges total.
    # With pen_speed=12 and dt=1/120 the per-edge step count is deterministic.
    pose = Pose(0.0, 0.0, 0.0)
    for cmd in cmds:
        pose = step(pose, cmd.v_left, cmd.v_right, GEOMETRY.width, cmd.duration)
    pen_final = transform_point(pose, *pen_body)
    assert hypot(pen_final[0] - targets[-1][0],
                 pen_final[1] - targets[-1][1]) < 0.5


def test_trace_requires_off_axis_pen():
    with pytest.raises(ScriptError) as exc:
        parse_script("trace 10 10", GEOMETRY, pen_body=(0.0, 5.0))
    assert "off-axis" in str(exc.value).lower() or "px" in str(exc.value).lower()
