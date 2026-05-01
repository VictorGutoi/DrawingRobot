"""Figure 7 — Three pen-path strategies on a 30 cm square.
Simulates goto, line_to and trace plans with the same off-axis pen and
plots the resulting pen traces side by side.
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


def simulate(script_text, geometry, pen_body, start_pose):
    cmds = parse_script(script_text, geometry, pen_body=pen_body)
    pose = Pose(start_pose.x, start_pose.y, start_pose.theta)
    px, py = pen_body
    pen_xs, pen_ys = [], []
    body_xs, body_ys = [pose.x], [pose.y]
    c, s = cos(pose.theta), sin(pose.theta)
    pen_xs.append(pose.x + px * c - py * s)
    pen_ys.append(pose.y + px * s + py * c)
    for cmd in cmds:
        pose = step(pose, cmd.v_left, cmd.v_right, geometry.width, cmd.duration)
        body_xs.append(pose.x); body_ys.append(pose.y)
        c, s = cos(pose.theta), sin(pose.theta)
        pen_xs.append(pose.x + px * c - py * s)
        pen_ys.append(pose.y + px * s + py * c)
    return np.array(pen_xs), np.array(pen_ys), np.array(body_xs), np.array(body_ys)


def main():
    rd.setup_style()
    fig, axes = rd.figure(width=7.4, height=2.8, subplots=(1, 3))

    geometry = RobotGeometry(width=20.4, length=23.2, wheel_offset=8.8)
    pen_body = (14.4, 0.0)
    # parse_script plans from pose=(0,0,0); pen starts at (px, py).
    start = Pose(0.0, 0.0, 0.0)
    px, py = pen_body

    scripts = [
        ('goto',    'goto 30 0\ngoto 30 30\ngoto 0 30\ngoto -10 0'),
        ('line_to', 'line_to 30 0\nline_to 30 30\nline_to 0 30\nline_to 0 0'),
        ('trace',   'trace 30 0 30 30 0 30 0 0'),
    ]

    pen_start_x, pen_start_y = px, py  # theta=0
    for ax, (name, src) in zip(axes, scripts):
        pen_x, pen_y, body_x, body_y = simulate(src, geometry, pen_body, start)
        # Target polyline: each script aims its pen at user vertices, but the
        # path begins from wherever the pen currently is (here: (px, py)).
        if name == 'goto':
            tx = [pen_start_x, 30, 30,  0, -10]
            ty = [pen_start_y,  0, 30, 30,   0]
        elif name == 'line_to':
            tx = [pen_start_x, 30, 30, 0, 0]
            ty = [pen_start_y,  0, 30, 30, 0]
        else:
            tx = [pen_start_x, 30, 30, 0, 0]
            ty = [pen_start_y,  0, 30, 30, 0]
        ax.plot(tx, ty, color=rd.COLOR_GUIDE, lw=1.0, ls='--')
        ax.plot(body_x, body_y,
                color=rd.COLOR_PRIMARY, lw=0.9, alpha=0.6,
                label='body $M$')
        ax.plot(pen_x, pen_y,
                color=rd.COLOR_SECONDARY, lw=1.4,
                label='pen')
        ax.set_title(f'{name}', fontsize=10)
        ax.set_aspect('equal')
        ax.grid(True, lw=0.4, alpha=0.4)
        ax.set_xlim(-15, 40)
        ax.set_ylim(-20, 40)
        ax.tick_params(labelsize=8)

    axes[0].set_ylabel(r'$y$ (cm)')
    for ax in axes:
        ax.set_xlabel(r'$x$ (cm)')
    axes[-1].legend(loc='lower right', framealpha=0.95, fontsize=8)

    fig.tight_layout()
    rd.save(fig, '01.7-square-comparison.pdf')


if __name__ == '__main__':
    main()
