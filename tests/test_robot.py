from math import isclose

import pytest

from drawingrobot.robot import RobotGeometry


def test_perimeter():
    g = RobotGeometry(width=20.0, length=30.0, wheel_offset=10.0)
    assert isclose(g.perimeter, 100.0)


def test_chassis_corners_with_offset_wheels():
    g = RobotGeometry(width=20.0, length=30.0, wheel_offset=10.0)
    corners = g.chassis_corners()
    assert corners[0] == (-10.0, -10.0)
    assert corners[1] == (20.0, -10.0)
    assert corners[2] == (20.0, 10.0)
    assert corners[3] == (-10.0, 10.0)


def test_pen_offset_at_corners():
    g = RobotGeometry(width=20.0, length=30.0, wheel_offset=10.0)
    assert g.pen_offset(0.0) == (-10.0, -10.0)
    assert g.pen_offset(30.0) == (20.0, -10.0)
    assert g.pen_offset(50.0) == (20.0, 10.0)
    assert g.pen_offset(80.0) == (-10.0, 10.0)


def test_pen_offset_midpoints_lie_on_outline():
    g = RobotGeometry(width=20.0, length=30.0, wheel_offset=10.0)
    assert g.pen_offset(15.0) == (5.0, -10.0)
    assert g.pen_offset(40.0) == (20.0, 0.0)
    assert g.pen_offset(65.0) == (5.0, 10.0)
    assert g.pen_offset(90.0) == (-10.0, 0.0)


def test_pen_offset_wraps_around_perimeter():
    g = RobotGeometry(width=20.0, length=30.0, wheel_offset=10.0)
    assert g.pen_offset(g.perimeter) == g.pen_offset(0.0)
    assert g.pen_offset(g.perimeter + 15.0) == g.pen_offset(15.0)


def test_invalid_dimensions_raise():
    with pytest.raises(ValueError):
        RobotGeometry(width=0.0, length=10.0, wheel_offset=0.0)
    with pytest.raises(ValueError):
        RobotGeometry(width=10.0, length=10.0, wheel_offset=11.0)
    with pytest.raises(ValueError):
        RobotGeometry(width=10.0, length=10.0, wheel_offset=-1.0)
