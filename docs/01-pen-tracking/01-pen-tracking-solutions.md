---
title: "Solutions — Lesson 01"
subtitle: "Pen-Path Tracking on a Differential-Drive Robot · DrawingRobot"
date: \today
---

\fancyhead[L]{\small\itshape DrawingRobot}

These are full worked solutions for every exercise in `01-pen-tracking.md`.
Try the exercises before peeking; the value of an exercise is in attempting it,
even unsuccessfully.

---

## Inline exercises

### Exercise 1.1 — Body $v$, $\omega$, ICR for $v_l = 8$, $v_r = 12$ cm/s.

Given $v_l = 8$ cm/s, $v_r = 12$ cm/s, $W = 20.4$ cm.

(a) Linear speed:
$$v = \tfrac{1}{2}(v_l + v_r) = \tfrac{1}{2}(8 + 12) = 10\;\text{cm/s}.$$

(b) Angular speed:
$$\omega = \dfrac{v_r - v_l}{W} = \dfrac{12 - 8}{20.4} = \dfrac{4}{20.4} \approx 0.196\;\text{rad/s}.$$

(c) ICR distance from $M$:
$$R = \dfrac{v}{\omega} = \dfrac{10}{0.196} \approx 51\;\text{cm}.$$

Sign of $\omega$ is positive, which by convention means CCW rotation. Looking from above with $x$ forward and $y$ to the left, CCW rotation about $M$ is a left turn. So **the body is turning left**, on a circle of radius $\approx 51$ cm centred to the left of $M$.

*Sanity check:* the right wheel is moving faster than the left wheel, so the body should yaw counter-clockwise (left). ✓

---

### Exercise 1.2 — Compute $\det J(\theta)$.

Starting from
$$J(\theta) = \begin{pmatrix}\cos\theta & -(p_x\sin\theta + p_y\cos\theta) \\ \sin\theta & \;\;p_x\cos\theta - p_y\sin\theta\end{pmatrix},$$

apply the $2 \times 2$ determinant formula $\det = ad - bc$:
$$\det J = \cos\theta\,(p_x\cos\theta - p_y\sin\theta) - \sin\theta\,\bigl(-(p_x\sin\theta + p_y\cos\theta)\bigr).$$

Expand:
$$= p_x\cos^2\theta - p_y\sin\theta\cos\theta + p_x\sin^2\theta + p_y\sin\theta\cos\theta.$$

The $p_y\sin\theta\cos\theta$ terms cancel, and $\cos^2\theta + \sin^2\theta = 1$, so:
$$\boxed{\;\det J(\theta) = p_x\;}$$

Independent of $\theta$ and of $p_y$.

*Sanity check:* in the special case $\theta = 0$, $J = \begin{pmatrix}1 & -p_y \\ 0 & p_x\end{pmatrix}$, an upper triangular matrix whose determinant is the product of the diagonal: $1 \cdot p_x = p_x$. ✓

---

### Exercise 1.3 — Pen at $(0, 10.2)$: is `trace` available?

The pen sits on the wheel-axis line ($p_x = 0$), specifically directly on top of the left wheel. By the result of Exercise 1.2, $\det J = p_x = 0$, so $J$ is singular at every heading and $J^{-1}$ does not exist. The `trace` primitive depends on inverting $J$ at every timestep, so **it cannot work for this pen position**.

The codebase explicitly rejects `trace` for $p_x = 0$ (see `_plan_trace` in `drawingrobot/script.py`). The alternative is `line_to`, which plans rotate-translate-rotate setup + forward and does not invert $J$ — and in this special case the setup translation $\Delta = (R(\theta_\text{cur}) - R(\theta_\text{new}))(p_x, p_y)^\top$ has the wheel-axis-aligned $p_y$ component preserved, so the corner curvature is concentrated at the vertex with magnitude $|p_y(1 - \cos\Delta\theta)|$ for the $90^\circ$ case, $\approx |p_y|$.

---

### Exercise 1.4 — Single-arc bound for $(p_x, p_y) = (14.4, 0)$.

(a) The single-arc lemma gives $r_\text{pen} \ge |p_x| = 14.4$ cm. Saturated when the ICR is placed at $R = p_y = 0$, i.e. directly under $M$ — but that is exactly in-place rotation, which gives $r_\text{pen} = |p_\text{body}| = \sqrt{14.4^2 + 0^2} = 14.4$ cm. So **14.4 cm**.

(b) `goto` plans a rotate-then-forward at each leg. The rotation is in-place, so its pen radius is $|p_\text{body}| = 14.4$ cm. **14.4 cm at every corner.**

(c) The square's edge length is $30$ cm but each corner arc has radius $14.4$ cm — almost half the edge. The arcs span a substantial fraction of each leg, so the pen path between corners is dominated by a large fillet rather than a straight segment. The user sees a smooth blob rather than a square. (`line_to` and `trace` are designed precisely to fix this — `line_to` by localising the corner curvature at the vertex, and `trace` by sidestepping the single-arc lemma entirely.)

*Sanity check:* the simulator screenshot in §5 (Figure 1.7, left panel) shows exactly this — `goto` produces a chaotic shape because the corner arcs span the legs. ✓

---

### Exercise 1.5 — $(v, \omega)$ for $\dot P_\text{des} = (0, 10)$ at $\theta = 0$, $p = (14.4, 0)$.

Pen offset $(p_x, p_y) = (14.4, 0)$, heading $\theta = 0$, so $\sin\theta = 0$, $\cos\theta = 1$, and
$$a = p_x\sin\theta + p_y\cos\theta = 0, \qquad b = p_x\cos\theta - p_y\sin\theta = 14.4.$$

Apply the inverse-Jacobian formula with $\dot P_\text{des} = (\dot P_x, \dot P_y) = (0, 10)$:
$$v = \dfrac{b\,\dot P_x + a\,\dot P_y}{p_x} = \dfrac{14.4 \cdot 0 + 0 \cdot 10}{14.4} = 0,$$
$$\omega = \dfrac{-\sin\theta\,\dot P_x + \cos\theta\,\dot P_y}{p_x} = \dfrac{0 \cdot 0 + 1 \cdot 10}{14.4} = \dfrac{10}{14.4} \approx 0.694\;\text{rad/s}.$$

So $(v, \omega) = (0,\;0.694)$ rad/s. **The body spins in place.**

That looks counter-intuitive — the pen is supposed to move sideways in the world, why is the body not translating? The answer: with $\theta = 0$ the body's heading is along world $+x$, and a pen at $(p_x, p_y) = (14.4, 0)$ sits in front of $M$ along that heading. The non-holonomic constraint forbids $M$ from moving in world $+y$ without a rotation. The only way the pen-tip moves in world $+y$ at this instant is by the body rotating about $M$ — which swings the pen-tip perpendicular to $\overrightarrow{MP}$, i.e. exactly in the world $+y$ direction at this configuration.

*Sanity check:* a pure rotation $\omega = 10/14.4$ rad/s about $M$ moves a point at distance $14.4$ from $M$ at tangential speed $\omega \cdot 14.4 = 10$ cm/s perpendicular to $\overrightarrow{MP}$. Since $\overrightarrow{MP}$ is along $+x$, the tangential direction is $+y$. ✓

---

### Exercise 1.6 — Pen at $(0.5, 7)$: `trace` available, but…

(a) Yes, $p_x = 0.5 \neq 0$, so $\det J = 0.5 \neq 0$, $J$ is invertible, `trace` is available.

(b) The undesirable issue is **gain**. The inverse Jacobian has $1/p_x$ as a prefactor: with $p_x = 0.5$ cm, even moderate commanded pen velocities produce large $\omega$ — for $\dot P_\text{des} = 10$ cm/s in the unfavourable direction, $\omega = O(\dot P / p_x) = O(20)$ rad/s, which is well above the simulator's default $\pi$ rad/s and certainly above any plausible physical wheel speed. The wheels saturate, the inverse Jacobian's request is not met, and the pen drifts off the polyline. In short: **`trace` is well-defined but the actuator authority is wrong** — the controller asks for body rotations the wheels cannot deliver.

---

## End-of-chapter

### Exercise 1.E1 — Diff-drive expressions and limits.

For wheel speeds $v_l, v_r$ on a chassis of width $W$:
$$v = \tfrac{1}{2}(v_l + v_r), \qquad \omega = \dfrac{v_r - v_l}{W}, \qquad R = \dfrac{v}{\omega} = \dfrac{W}{2}\,\dfrac{v_l + v_r}{v_r - v_l}.$$

Limits:

- $v_l = v_r = v_0$ ⇒ $\omega = 0$, $v = v_0$, $R = \infty$. Body translates straight along its heading at speed $v_0$. ✓
- $v_l = -v_r = -v_0$ ⇒ $v = 0$, $\omega = 2v_0/W$, $R = 0$. Body spins in place about $M$ at angular speed $2v_0/W$. ✓
- $v_l = 0$, $v_r = v_0$ ⇒ $v = v_0/2$, $\omega = v_0/W$, $R = W/2$. Body pivots about the *left* wheel (which is sitting still). The radius $R = W/2$ confirms the ICR sits exactly at the left wheel position. ✓

---

### Exercise 1.E2 — Pen position at world pose $(5, 2, \pi/4)$, body offset $(10, -3)$.

Apply
$$P_x = x + p_x\cos\theta - p_y\sin\theta, \qquad P_y = y + p_x\sin\theta + p_y\cos\theta.$$

With $\theta = \pi/4$, $\cos\theta = \sin\theta = 1/\sqrt 2 \approx 0.7071$:
$$P_x = 5 + 10 \cdot 0.7071 - (-3)\cdot 0.7071 = 5 + 7.071 + 2.121 = 14.19\;\text{cm}.$$
$$P_y = 2 + 10 \cdot 0.7071 + (-3)\cdot 0.7071 = 2 + 7.071 - 2.121 = 6.95\;\text{cm}.$$

So $\boxed{\;P \approx (14.19,\,6.95)\;\text{cm}\;}$.

*Sanity check:* the pen offset $(10, -3)$ has magnitude $\sqrt{100 + 9} \approx 10.44$ cm; it should land at distance $10.44$ from $M = (5, 2)$. Computing: $\sqrt{(14.19 - 5)^2 + (6.95 - 2)^2} = \sqrt{84.5 + 24.5} = \sqrt{109.0} \approx 10.44$ cm. ✓

---

### Exercise 1.E3 — No non-holonomic constraint on $P$ when $p_x \neq 0$.

The body constraint reads $\dot x \sin\theta - \dot y \cos\theta = 0$ at every instant: the wheel midpoint $M$ has zero velocity in the body $\hat y_b$-direction.

Suppose, for contradiction, the pen $P$ obeys an analogous constraint of the form
$$\dot P_x f(\theta) + \dot P_y g(\theta) = 0$$
for some non-trivial $(f, g)$. Then the row vector $(f, g)$ must annihilate $J(\theta)\,(v, \omega)^\top$ for *every* admissible $(v, \omega)$. So $(f, g)$ must be in the *left null space* of $J$. But $\det J = p_x$ — and if $p_x \neq 0$, $J$ has full rank, so its left null space is $\{0\}$. The only $(f, g)$ that works is the zero vector, i.e. there is **no constraint**.

Physical interpretation: when the pen sits off the wheel-axis line, $M$'s sideways immobility is partly compensated by the rotation of the body about $M$, which translates the pen sideways from $M$'s heading. The pen velocity gets a 2-D worth of authority — a translation contribution from $v$ along $\hat x_b$ and a rotation contribution from $\omega$ perpendicular to $\overrightarrow{MP}$ — and these two basis vectors are linearly independent precisely when $\overrightarrow{MP}$ has a non-zero body $\hat x_b$-component (i.e. $p_x \neq 0$).

When $p_x = 0$ the rotation contribution becomes parallel to $\hat y_b$ — both basis vectors collapse to a 1-D span, and the pen recovers a non-holonomic constraint (its velocity must lie along $\hat y_b$ from $M$'s perspective).

---

### Exercise 1.E4 — Verify $J\,J^{-1} = I_2$.

Let $a = p_x\sin\theta + p_y\cos\theta$, $b = p_x\cos\theta - p_y\sin\theta$. Then
$$J = \begin{pmatrix}\cos\theta & -a \\ \sin\theta & b\end{pmatrix}, \qquad J^{-1}\stackrel{?}{=}\dfrac{1}{p_x}\begin{pmatrix}b & a \\ -\sin\theta & \cos\theta\end{pmatrix}.$$

Compute $J\,J^{-1}$ entry by entry. The $(1, 1)$ entry:
$$\dfrac{1}{p_x}\bigl[\cos\theta\cdot b + (-a)\cdot(-\sin\theta)\bigr] = \dfrac{1}{p_x}\bigl[b\cos\theta + a\sin\theta\bigr].$$
Substituting,
$$b\cos\theta + a\sin\theta = (p_x\cos\theta - p_y\sin\theta)\cos\theta + (p_x\sin\theta + p_y\cos\theta)\sin\theta$$
$$= p_x(\cos^2\theta + \sin^2\theta) + p_y(-\sin\theta\cos\theta + \cos\theta\sin\theta) = p_x.$$
So the $(1, 1)$ entry is $p_x/p_x = 1$. ✓

The $(1, 2)$ entry:
$$\dfrac{1}{p_x}\bigl[\cos\theta\cdot a + (-a)\cdot \cos\theta\bigr] = 0. ✓$$

The $(2, 1)$ entry:
$$\dfrac{1}{p_x}\bigl[\sin\theta\cdot b + b\cdot(-\sin\theta)\bigr] = 0. ✓$$

The $(2, 2)$ entry:
$$\dfrac{1}{p_x}\bigl[\sin\theta\cdot a + b\cdot \cos\theta\bigr] = \dfrac{1}{p_x}\bigl[a\sin\theta + b\cos\theta\bigr] = \dfrac{p_x}{p_x} = 1. ✓$$

So $J\,J^{-1} = I_2$, confirming the formula.

---

### Exercise 1.E5 — Saturation at a sharp corner, $p_x = 14.4$, $v_\text{pen} = 12$, $\Delta t = 1/120$.

(a) Heading change required so that the pen-tip can pivot from moving in $+x$ to moving in $+y$. With $\theta_0 = 0$ and pen at body offset $(14.4, 0)$, the pen velocity column 1 of $J$ is $(\cos\theta, \sin\theta) = (1, 0)$ at $\theta = 0$ — perfect for $\dot P = (12, 0)$. After the corner we want $\dot P = (0, 12)$, which is achieved at $\theta_1 = \pi/2$ (heading along $+y$). So the body must change heading by $|\Delta\theta| = \pi/2 \approx 1.571$ rad.

(b) Spread over a single timestep $\Delta t = 1/120$ s, this requires
$$|\omega|_\text{peak} \approx \dfrac{|\Delta\theta|}{\Delta t} = \dfrac{\pi/2}{1/120} = 60\pi \approx 188.5\;\text{rad/s}.$$

(c) The simulator's default in-place rotation speed is $\omega_\text{rot} = \pi$ rad/s, which corresponds to wheel-tip speeds $v_l = -\omega W/2 = -\pi \cdot 10.2 \approx -32$ cm/s on each side. The peak $|\omega|$ at the corner exceeds this by a factor of $60$, so **yes, the wheels saturate severely** at the corner.

What actually happens in the simulator: the per-timestep wheel-velocity demand is enormous, and (depending on whether the simulator clips wheel speeds) either (i) the pose-integration step takes a single huge $\omega\Delta t$ that swings the body too much, or (ii) the wheels saturate, the demanded $\Delta\theta$ is not delivered, and the pen "rounds" the corner with a small fillet of radius proportional to the missing rotation. In practice we don't notice this in the trace plots because the integration is done with the formulas as given — but if the codebase later adds a wheel-speed clip, this corner will be the first place the trace primitive visibly degrades.

The right cure is in §4's "more robust planner" sketch: bound $|\omega|$ to a ceiling and round corners only when it would otherwise saturate.

---

### Exercise 1.E6 — Two arcs cannot beat $|p_x|$.

Each arc command, by the single-arc lemma of §3, paints a pen circle of radius $\ge |p_x|$ — the bound is on the *instantaneous curvature radius* of the pen during that command. So during the first command, the pen's curvature radius is $\ge |p_x|$; during the second command, also $\ge |p_x|$. The transition between the two commands is instantaneous (the command boundary), so the pen's pose at the join is just whatever the first command produced — there is no instantaneous "kink" allowed in the pen's curvature, only a discontinuity in the curvature value.

So the pen's radius of curvature is bounded below by $|p_x|$ at every instant during the two-arc composition. The smallest pen circle inscribed in such a path has radius $\ge |p_x|$. The bound is unchanged.

By induction, the same holds for any **finite** composition of constant-input commands: the worst-case pen curvature radius across the whole composition is $\ge |p_x|$.

What `trace` does differently: it does not issue constant-input commands. It updates $(v, \omega)$ at every timestep, and at the corner specifically it asks for instantaneous curvature radius approaching zero. The pen's *average* curvature radius over a timestep is still finite (limited by $v_\text{pen}\Delta t$), but the bound $r \ge |p_x|$ no longer applies because the inputs are no longer constant. Many small commands per edge — at $\Delta t = 1/120$ s, around $360$ commands per $30$ cm leg — give the pen velocity-discontinuity authority that the single-arc lemma denied.
