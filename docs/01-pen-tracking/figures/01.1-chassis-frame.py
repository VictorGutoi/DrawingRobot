"""Figure 1 — Chassis, wheels, pen offset, body frame.
Shows the rigid-body geometry: chassis rectangle, wheels at offset along the
two long sides, wheel-axis midpoint M, body axes, pen P at offset (px, py)
on the outline.
"""
import rtlib_diagrams as rd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch


def main():
    rd.setup_style()
    fig, ax = rd.figure(width=6.0, height=4.2)

    length, width = 23.2, 20.4
    wheel_offset_from_back = 8.8
    wheel_w, wheel_h = 6.6, 1.6

    # Chassis (centered for clarity)
    ax.add_patch(Rectangle((-length / 2, -width / 2), length, width,
                           fill=False, lw=1.6, ec=rd.COLOR_AXIS))

    # Wheels: directly opposite, on the two long sides, at wheel_offset_from_back
    wx = -length / 2 + wheel_offset_from_back
    for sign in (-1, 1):
        ax.add_patch(Rectangle((wx - wheel_w / 2, sign * width / 2 - wheel_h / 2),
                               wheel_w, wheel_h,
                               fc=rd.COLOR_AXIS, ec=rd.COLOR_AXIS))

    # Wheel-axis line and midpoint M
    ax.plot([wx, wx], [-width / 2, width / 2],
            color=rd.COLOR_GUIDE, lw=1.0, ls='--')
    ax.plot(wx, 0, 'o', color=rd.COLOR_AXIS, ms=4)
    ax.text(wx + 0.6, 0.4, r'$M$', fontsize=11, color=rd.COLOR_AXIS)

    # Pen P on the outline (front-mid for the example, slightly off-axis)
    px_b, py_b = length / 2 - wx, -3.0  # body-frame offset from M
    pen_world = (wx + px_b, py_b)
    ax.plot(*pen_world, 'o', color=rd.COLOR_SECONDARY, ms=6)
    ax.text(pen_world[0] + 0.4, pen_world[1] - 1.4, r'$P$ (pen)',
            fontsize=11, color=rd.COLOR_SECONDARY)

    # Body x-axis (heading) from M
    ax.add_patch(FancyArrowPatch((wx, 0), (wx + 6.5, 0),
                                 arrowstyle='-|>', mutation_scale=14,
                                 color=rd.COLOR_PRIMARY, lw=1.2))
    ax.text(wx + 6.8, 0.4, r'$\hat{x}_b$', color=rd.COLOR_PRIMARY, fontsize=11)

    # Body y-axis (wheel axis direction)
    ax.add_patch(FancyArrowPatch((wx, 0), (wx, 6.5),
                                 arrowstyle='-|>', mutation_scale=14,
                                 color=rd.COLOR_PRIMARY, lw=1.2))
    ax.text(wx + 0.4, 6.6, r'$\hat{y}_b$', color=rd.COLOR_PRIMARY, fontsize=11)

    # Pen offset vector from M to P
    ax.add_patch(FancyArrowPatch((wx, 0), pen_world,
                                 arrowstyle='-|>', mutation_scale=12,
                                 color=rd.COLOR_SECONDARY, lw=1.1, ls='-'))
    ax.text((wx + pen_world[0]) / 2 + 0.3, (pen_world[1]) / 2 - 1.0,
            r'$(p_x, p_y)$', color=rd.COLOR_SECONDARY, fontsize=10)

    # Wheel labels
    ax.text(wx, -width / 2 - 1.6, 'right wheel',
            ha='center', va='top', fontsize=9, color=rd.COLOR_AXIS)
    ax.text(wx, width / 2 + 1.6, 'left wheel',
            ha='center', va='bottom', fontsize=9, color=rd.COLOR_AXIS)
    ax.annotate('', xy=(wx - 4, -width / 2), xytext=(wx - 4, width / 2),
                arrowprops=dict(arrowstyle='<|-|>', color=rd.COLOR_GUIDE, lw=0.9))
    ax.text(wx - 4.4, 0, r'$W$ (wheelbase)',
            ha='right', va='center', fontsize=9, color=rd.COLOR_GUIDE)

    # World axes (offset)
    ox, oy = -length / 2 - 4.5, -width / 2 - 1.5
    ax.add_patch(FancyArrowPatch((ox, oy), (ox + 4, oy),
                                 arrowstyle='-|>', mutation_scale=12,
                                 color=rd.COLOR_GUIDE, lw=1.0))
    ax.add_patch(FancyArrowPatch((ox, oy), (ox, oy + 4),
                                 arrowstyle='-|>', mutation_scale=12,
                                 color=rd.COLOR_GUIDE, lw=1.0))
    ax.text(ox + 4.3, oy, r'$x$ (world)', color=rd.COLOR_GUIDE, fontsize=9, va='center')
    ax.text(ox, oy + 4.3, r'$y$ (world)', color=rd.COLOR_GUIDE, fontsize=9, ha='center')

    ax.set_xlim(-length / 2 - 5.5, length / 2 + 4.5)
    ax.set_ylim(-width / 2 - 4.0, width / 2 + 4.0)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ('top', 'right', 'left', 'bottom'):
        ax.spines[s].set_visible(False)

    rd.save(fig, '01.1-chassis-frame.pdf')


if __name__ == '__main__':
    main()
