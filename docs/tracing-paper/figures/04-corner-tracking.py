"""Figure 4 — Pen tracks a sharp corner; the body weaves underneath.
Simulates a single 90° corner with the trace primitive (feedback linearisation)
and overlays the pen path (sharp) and the body (M) path (smooth swing).
"""
import sys
from math import cos, sin
import numpy as np
import rtlib_diagrams as rd
import matplotlib.pyplot as plt

sys.path.insert(0, '/Users/victor/Documents/_Projects/DrawingRobot')

from drawingrobot.kinematics import Pose, step
from drawingrobot.robot import RobotGeometry
from drawingrobot.script import parse_script


def main():
    rd.setup_style()
    fig, ax = rd.figure(width=6.0, height=4.4)

    geometry = RobotGeometry(width=20.4, length=23.2, wheel_offset=8.8)
    pen_body = (14.4, 0.0)  # off-axis pen, on the chassis front-mid

    # Just the corner: come in along +x, turn 90 deg to +y.
    # parse_script plans assuming pose starts at (0,0,0), so pen starts at
    # (px, py) in world coords; we run the same simulation here.
    cmds = parse_script("trace 30 0 30 30", geometry, pen_body=pen_body)

    pose = Pose(0.0, 0.0, 0.0)
    body_xs, body_ys, pen_xs, pen_ys = [pose.x], [pose.y], [], []
    px, py = pen_body
    pen0 = (pose.x + px * cos(pose.theta) - py * sin(pose.theta),
            pose.y + px * sin(pose.theta) + py * cos(pose.theta))
    pen_xs.append(pen0[0]); pen_ys.append(pen0[1])

    for cmd in cmds:
        pose = step(pose, cmd.v_left, cmd.v_right, geometry.width, cmd.duration)
        body_xs.append(pose.x); body_ys.append(pose.y)
        c, s = cos(pose.theta), sin(pose.theta)
        pen_xs.append(pose.x + px * c - py * s)
        pen_ys.append(pose.y + px * s + py * c)

    # Plot pen path (sharp polyline target). The planner prepends the current
    # pen position to the user-given vertices, so the actual targeted polyline
    # is [pen_start, (30,0), (30,30)].
    pen_start = (pose.x + px, pose.y + py)  # theta=0 at start
    ax.plot([pen_start[0], 30, 30],
            [pen_start[1], 0, 30],
            color=rd.COLOR_GUIDE, lw=1.0, ls='--', label='pen target polyline')
    ax.plot(pen_xs, pen_ys,
            color=rd.COLOR_SECONDARY, lw=1.8, label='pen actual')
    ax.plot(body_xs, body_ys,
            color=rd.COLOR_PRIMARY, lw=1.6, label='body $M$ (wheel midpoint)')

    # Mark a few snapshots showing pen offset from body
    snapshot_idxs = [0, len(body_xs) // 4, len(body_xs) // 2,
                     3 * len(body_xs) // 4, len(body_xs) - 1]
    for idx in snapshot_idxs:
        ax.plot([body_xs[idx], pen_xs[idx]],
                [body_ys[idx], pen_ys[idx]],
                color=rd.COLOR_GUIDE, lw=0.8, alpha=0.7)
        ax.plot(body_xs[idx], body_ys[idx], 'o', color=rd.COLOR_PRIMARY, ms=4)
        ax.plot(pen_xs[idx], pen_ys[idx], 'o', color=rd.COLOR_SECONDARY, ms=4)

    ax.set_xlabel(r'$x$ (cm)')
    ax.set_ylabel(r'$y$ (cm)')
    ax.set_aspect('equal')
    ax.legend(loc='lower right', framealpha=0.95, fontsize=9)
    ax.grid(True, lw=0.4, alpha=0.4)
    ax.set_xlim(-5, 35)
    ax.set_ylim(-5, 35)

    rd.save(fig, '04-corner-tracking.pdf')


if __name__ == '__main__':
    main()
