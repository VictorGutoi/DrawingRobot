"""Figure 5 — Lower bound on single-arc corner radius.
Illustrates that an in-place rotation (or any single arc command) sweeps a
pen circle whose radius is at least |px|; the only zero-radius positions
are the two wheel locations on the chassis outline.
"""
import numpy as np
import rtlib_diagrams as rd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


def main():
    rd.setup_style()
    fig, ax = rd.figure(width=5.6, height=4.0)

    L, W = 23.2, 20.4
    wx = 0.0  # wheel-axis at body x = 0

    # Chassis (body frame, M at origin)
    ax.add_patch(Rectangle((-W / 2 + (W - L) / 2, -W / 2),  # placeholder
                           L, W, fill=False, ec='none'))
    # Use real coords: x in [-wheel_offset, length-wheel_offset], y in [-W/2, W/2]
    wheel_offset = 8.8
    front_x = L - wheel_offset
    back_x = -wheel_offset
    ax.add_patch(Rectangle((back_x, -W / 2), L, W,
                           fill=False, ec=rd.COLOR_AXIS, lw=1.4))

    # Wheel-axis line (px = 0)
    ax.plot([0, 0], [-W / 2 - 1, W / 2 + 1],
            color=rd.COLOR_PRIMARY, lw=1.4, ls='--',
            label=r'wheel-axis line ($p_x=0$)')
    ax.plot(0, W / 2, 's', color=rd.COLOR_PRIMARY, ms=8)
    ax.plot(0, -W / 2, 's', color=rd.COLOR_PRIMARY, ms=8)

    # Several pen positions on the outline; for each, swept circle of radius |pen_body|
    pen_positions = [
        (front_x, 0.0),          # front-mid
        (front_x, W / 2 - 2),    # front-right area
        (back_x, 0.0),           # back-mid
        (front_x - 5, W / 2),    # along the side
    ]

    for px, py in pen_positions:
        r = np.hypot(px, py)
        ax.add_patch(Circle((0, 0), r, fill=False, ec=rd.COLOR_SECONDARY,
                            lw=0.9, alpha=0.6, ls=':'))
        ax.plot(px, py, 'o', color=rd.COLOR_SECONDARY, ms=5)

    # Label the front-mid case explicitly
    px_lbl, py_lbl = front_x, 0.0
    ax.annotate(r'pen swept circle: $r=\sqrt{p_x^2+p_y^2}$',
                xy=(np.hypot(px_lbl, py_lbl), 0), xytext=(20, 8),
                fontsize=9, color=rd.COLOR_SECONDARY,
                arrowprops=dict(arrowstyle='->', color=rd.COLOR_SECONDARY, lw=0.8))

    ax.plot(0, 0, 'o', color=rd.COLOR_AXIS, ms=4)
    ax.text(0.4, 0.4, r'$M$', fontsize=10, color=rd.COLOR_AXIS)

    ax.text(-9.3, W / 2 + 1.2, 'left wheel',
            fontsize=9, color=rd.COLOR_PRIMARY, ha='center')
    ax.text(-9.3, -W / 2 - 2.2, 'right wheel',
            fontsize=9, color=rd.COLOR_PRIMARY, ha='center')

    ax.legend(loc='upper right', framealpha=0.95, fontsize=9)
    ax.set_aspect('equal')
    ax.set_xlim(-15, 30)
    ax.set_ylim(-18, 20)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ('top', 'right', 'left', 'bottom'):
        ax.spines[sp].set_visible(False)

    rd.save(fig, '01.5-min-radius.pdf')


if __name__ == '__main__':
    main()
