"""Figure 6 — Control loop: feedforward + P-feedback + inverse Jacobian.
Block diagram of the per-timestep tracking loop used by the trace primitive.
"""
import rtlib_diagrams as rd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


def box(ax, x, y, w, h, text, fc='#e8eef5', tc=None):
    if tc is None:
        tc = rd.COLOR_AXIS
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle='round,pad=0.02',
                                fc=fc, ec=rd.COLOR_AXIS, lw=1.0))
    ax.text(x + w / 2, y + h / 2, text,
            ha='center', va='center', fontsize=9.5, color=tc)


def arrow(ax, x1, y1, x2, y2, label=None, lab_offset=(0, 0.25)):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                 arrowstyle='-|>', mutation_scale=12,
                                 color=rd.COLOR_AXIS, lw=1.0))
    if label:
        ax.text((x1 + x2) / 2 + lab_offset[0],
                (y1 + y2) / 2 + lab_offset[1],
                label, fontsize=9, color=rd.COLOR_PRIMARY,
                ha='center', va='center')


def main():
    rd.setup_style()
    fig, ax = rd.figure(width=7.2, height=3.0)

    # Reference pen path
    box(ax, 0.0, 1.6, 1.7, 0.8, 'reference\n$p_{\\rm des}(t),\\,\\dot p_{\\rm des}$')

    # Sum junction (drawn as a small circle with +/-)
    sx, sy = 2.4, 2.0
    ax.add_patch(plt.Circle((sx, sy), 0.18, fc='white', ec=rd.COLOR_AXIS, lw=1.0))
    ax.text(sx, sy, '+', ha='center', va='center', fontsize=10)
    ax.text(sx + 0.18, sy - 0.35, '$-$', ha='center', va='center', fontsize=10)

    # Control law / inverse Jacobian (multi-line: matplotlib mathtext does
    # not support newlines inside $...$, so break each line separately).
    bx, by, bw, bh = 3.1, 1.6, 2.0, 0.8
    ax.add_patch(FancyBboxPatch((bx, by), bw, bh,
                                boxstyle='round,pad=0.02',
                                fc='#fff8e3', ec=rd.COLOR_AXIS, lw=1.0))
    ax.text(bx + bw / 2, by + bh * 0.72,
            r'$\dot p_{\rm cmd}=\dot p_{\rm des}+K_p(p_{\rm des}-p)$',
            ha='center', va='center', fontsize=8.5, color=rd.COLOR_AXIS)
    ax.text(bx + bw / 2, by + bh * 0.28,
            r'$(v,\,\omega)=J^{-1}\dot p_{\rm cmd}$',
            ha='center', va='center', fontsize=8.5, color=rd.COLOR_AXIS)

    # Diff drive map
    bx, by, bw, bh = 5.6, 1.6, 1.6, 0.8
    ax.add_patch(FancyBboxPatch((bx, by), bw, bh,
                                boxstyle='round,pad=0.02',
                                fc='#e8eef5', ec=rd.COLOR_AXIS, lw=1.0))
    ax.text(bx + bw / 2, by + bh * 0.7, r'$v_l = v - \omega W/2$',
            ha='center', va='center', fontsize=9, color=rd.COLOR_AXIS)
    ax.text(bx + bw / 2, by + bh * 0.3, r'$v_r = v + \omega W/2$',
            ha='center', va='center', fontsize=9, color=rd.COLOR_AXIS)

    # Plant: kinematics + pen output
    box(ax, 0.6, 0.0, 2.6, 0.8, 'differential-drive\nkinematics (state $x,y,\\theta$)',
        fc='#f3e8e8')
    box(ax, 4.0, 0.0, 2.6, 0.8, 'pen output\n$p = M + R(\\theta)\\,(p_x,p_y)$',
        fc='#f3e8e8')

    # Arrows
    arrow(ax, 1.7, 2.0, sx - 0.18, 2.0, label='$\\dot p_{\\rm des}$')
    arrow(ax, sx + 0.18, 2.0, 3.1, 2.0)
    arrow(ax, 5.1, 2.0, 5.6, 2.0, label='$(v,\\omega)$')
    arrow(ax, 7.2, 2.0, 7.6, 2.0)
    ax.text(7.62, 2.0, 'wheels', fontsize=9, va='center')

    # Feedback path: pen position p back to sum
    arrow(ax, 6.6, 0.4, 7.4, 0.4)
    arrow(ax, 7.4, 0.4, 7.4, 2.0)
    arrow(ax, 7.4, 2.0, 7.4, 2.6)
    arrow(ax, 7.4, 2.6, sx, 2.6)
    arrow(ax, sx, 2.6, sx, sy + 0.18, label='$p$', lab_offset=(0.25, 0.0))

    # Internal: kinematics out → pen output in
    arrow(ax, 3.2, 0.4, 4.0, 0.4, label='$x,y,\\theta$', lab_offset=(0, 0.22))

    # Wheels back to plant
    arrow(ax, 6.4, 1.6, 4.0, 0.8)

    ax.set_xlim(-0.2, 8.5)
    ax.set_ylim(-0.4, 3.2)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ('top', 'right', 'left', 'bottom'):
        ax.spines[sp].set_visible(False)

    rd.save(fig, '06-control-loop.pdf')


if __name__ == '__main__':
    main()
