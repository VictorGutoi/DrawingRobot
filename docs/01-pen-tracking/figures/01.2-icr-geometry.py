"""Figure 2 — ICR geometry for a single (constant v_l, v_r) command.
The instantaneous centre of rotation lies on the wheel-axis line. The
wheel-midpoint M sweeps an arc of radius R about the ICR; the pen sweeps
a concentric arc of radius |ICR - P|.
"""
import numpy as np
import rtlib_diagrams as rd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Arc, Circle


def main():
    rd.setup_style()
    fig, ax = rd.figure(width=6.2, height=4.4)

    # Robot pose centred at origin, theta = 30 deg
    L_chassis, W_chassis = 23.2, 20.4
    theta = np.deg2rad(20.0)
    c, s = np.cos(theta), np.sin(theta)

    # Chassis polygon, axis-aligned then rotated about M=(0,0)
    corners_b = np.array([[-L_chassis / 2, -W_chassis / 2],
                          [L_chassis / 2, -W_chassis / 2],
                          [L_chassis / 2, W_chassis / 2],
                          [-L_chassis / 2, W_chassis / 2],
                          [-L_chassis / 2, -W_chassis / 2]])
    R = np.array([[c, -s], [s, c]])
    corners_w = corners_b @ R.T
    ax.plot(corners_w[:, 0], corners_w[:, 1], color=rd.COLOR_AXIS, lw=1.4)

    # Wheel axis as a line through M perpendicular to heading
    wheel_axis_dir = np.array([-s, c])  # body y in world
    wa1 = wheel_axis_dir * (W_chassis / 2)
    wa2 = -wheel_axis_dir * (W_chassis / 2)
    ax.plot([wa1[0], wa2[0]], [wa1[1], wa2[1]],
            color=rd.COLOR_GUIDE, lw=1.0, ls='--')

    # ICR on the wheel-axis line, distance R_icr to the left
    R_icr = 18.0
    icr = wheel_axis_dir * R_icr
    ax.plot(*icr, 'x', color=rd.COLOR_SECONDARY, ms=10, mew=2)
    ax.text(icr[0] + 0.6, icr[1] + 0.6, r'ICR', color=rd.COLOR_SECONDARY, fontsize=11)

    # Body arc from M about ICR
    arc_radius_M = R_icr
    base_angle = np.degrees(np.arctan2(0 - icr[1], 0 - icr[0]))
    sweep = 35
    ax.add_patch(Arc(icr, 2 * arc_radius_M, 2 * arc_radius_M,
                     angle=0, theta1=base_angle - sweep, theta2=base_angle + sweep,
                     color=rd.COLOR_PRIMARY, lw=1.5))

    # Pen at front-corner offset, body (px, py) = (L/2, -W/2)
    px_b, py_b = L_chassis / 2 - 0, -W_chassis / 2 + 4
    pen_w = R @ np.array([px_b, py_b])
    ax.plot(*pen_w, 'o', color=rd.COLOR_SECONDARY, ms=6)
    ax.text(pen_w[0] + 0.5, pen_w[1] - 1.5, r'$P$', color=rd.COLOR_SECONDARY, fontsize=11)

    # Pen arc — concentric, radius |ICR - P|
    pen_radius = np.linalg.norm(pen_w - icr)
    base_angle_p = np.degrees(np.arctan2(pen_w[1] - icr[1], pen_w[0] - icr[0]))
    ax.add_patch(Arc(icr, 2 * pen_radius, 2 * pen_radius,
                     angle=0, theta1=base_angle_p - sweep, theta2=base_angle_p + sweep,
                     color=rd.COLOR_SECONDARY, lw=1.2, ls=':'))

    # M marker
    ax.plot(0, 0, 'o', color=rd.COLOR_AXIS, ms=4)
    ax.text(0.6, 0.6, r'$M$', fontsize=10, color=rd.COLOR_AXIS)

    # Radius from ICR to M
    ax.plot([icr[0], 0], [icr[1], 0], color=rd.COLOR_PRIMARY, lw=1.0)
    ax.text((icr[0]) / 2 - 1.0, (icr[1]) / 2 + 0.5, r'$R$',
            color=rd.COLOR_PRIMARY, fontsize=11)

    # Annotation for the formula
    ax.text(-22, 14,
            r'$R = \dfrac{W}{2}\,\dfrac{v_l + v_r}{v_r - v_l},\;\;'
            r'\omega = \dfrac{v_r - v_l}{W}$',
            fontsize=11, color=rd.COLOR_AXIS)

    ax.set_xlim(-25, 25)
    ax.set_ylim(-15, 22)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ('top', 'right', 'left', 'bottom'):
        ax.spines[sp].set_visible(False)

    rd.save(fig, '01.2-icr-geometry.pdf')


if __name__ == '__main__':
    main()
