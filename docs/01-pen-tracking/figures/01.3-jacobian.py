"""Figure 3 — Pen-velocity decomposition.
The pen velocity is the sum of the body translation (along heading) and a
rotation-induced sideways kick. The two columns of the Jacobian are these
two basis directions.
"""
import numpy as np
import rtlib_diagrams as rd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch


def main():
    rd.setup_style()
    fig, ax = rd.figure(width=6.2, height=4.0)

    # Body at origin, theta = 25 deg
    theta = np.deg2rad(25.0)
    c, s = np.cos(theta), np.sin(theta)
    L_chassis, W_chassis = 23.2, 20.4

    corners_b = np.array([[-L_chassis / 2, -W_chassis / 2],
                          [L_chassis / 2, -W_chassis / 2],
                          [L_chassis / 2, W_chassis / 2],
                          [-L_chassis / 2, W_chassis / 2],
                          [-L_chassis / 2, -W_chassis / 2]])
    Rmat = np.array([[c, -s], [s, c]])
    corners_w = corners_b @ Rmat.T
    ax.plot(corners_w[:, 0], corners_w[:, 1], color=rd.COLOR_AXIS, lw=1.3, alpha=0.6)

    # Body axes
    ax.add_patch(FancyArrowPatch((0, 0), (8 * c, 8 * s),
                                 arrowstyle='-|>', mutation_scale=12,
                                 color=rd.COLOR_PRIMARY, lw=1.0, alpha=0.7))
    ax.text(8 * c + 0.4, 8 * s, r'$\hat{x}_b$',
            color=rd.COLOR_PRIMARY, fontsize=10, alpha=0.8)

    # Pen at body offset (px, py)
    px_b, py_b = 9.0, -4.0
    pen = Rmat @ np.array([px_b, py_b])
    ax.plot(*pen, 'o', color=rd.COLOR_SECONDARY, ms=6)
    ax.text(pen[0] + 0.5, pen[1] - 1.4, r'$P$', color=rd.COLOR_SECONDARY, fontsize=11)

    # Translation contribution: v * (cos θ, sin θ)
    v_mag = 6.0
    trans_vec = np.array([v_mag * c, v_mag * s])
    ax.add_patch(FancyArrowPatch(pen, pen + trans_vec,
                                 arrowstyle='-|>', mutation_scale=14,
                                 color=rd.COLOR_PRIMARY, lw=1.6))
    ax.text(pen[0] + trans_vec[0] + 0.3,
            pen[1] + trans_vec[1] + 0.3,
            r'$v\,(\cos\theta,\sin\theta)$',
            color=rd.COLOR_PRIMARY, fontsize=10)

    # Rotation contribution: ω · (-px sin θ - py cos θ, px cos θ - py sin θ)
    omega = 0.6
    rot_vec = omega * np.array([-px_b * s - py_b * c,
                                 px_b * c - py_b * s])
    ax.add_patch(FancyArrowPatch(pen, pen + rot_vec,
                                 arrowstyle='-|>', mutation_scale=14,
                                 color=rd.COLOR_ACCENT, lw=1.6))
    ax.text(pen[0] + rot_vec[0] - 0.4,
            pen[1] + rot_vec[1] + 0.4,
            r'$\omega\,(\!-p_x\sin\theta-p_y\cos\theta,'
            r'\,p_x\cos\theta-p_y\sin\theta)$',
            color=rd.COLOR_ACCENT, fontsize=9, ha='right')

    # Resultant pen velocity (vector sum)
    res = trans_vec + rot_vec
    ax.add_patch(FancyArrowPatch(pen, pen + res,
                                 arrowstyle='-|>', mutation_scale=16,
                                 color=rd.COLOR_SECONDARY, lw=2.0))
    ax.text(pen[0] + res[0] + 0.4,
            pen[1] + res[1] - 0.7,
            r'$\dot{p} = J\,(v,\omega)^\top$',
            color=rd.COLOR_SECONDARY, fontsize=11)

    # Pen offset vector from M
    ax.add_patch(FancyArrowPatch((0, 0), pen,
                                 arrowstyle='-|>', mutation_scale=10,
                                 color=rd.COLOR_GUIDE, lw=0.9, ls='--'))

    ax.plot(0, 0, 'o', color=rd.COLOR_AXIS, ms=4)
    ax.text(-0.6, 0.6, r'$M$', color=rd.COLOR_AXIS, fontsize=10)

    ax.set_xlim(-15, 22)
    ax.set_ylim(-13, 16)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ('top', 'right', 'left', 'bottom'):
        ax.spines[sp].set_visible(False)

    rd.save(fig, '01.3-jacobian.pdf')


if __name__ == '__main__':
    main()
