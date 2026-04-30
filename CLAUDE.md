# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project purpose

Simulator for a pen-plotter-style robot built for a robotics competition. The physical robot is a rectangular chassis with two independently driven wheels and a pen rigidly attached somewhere on its outline; it draws by moving across a surface. This repo simulates that drawing on screen so we can explore which pen positions / wheel positions / chassis dimensions actually produce the shapes we need (lines, multi-angle paths, circles) before committing to a physical build.

The simulator is the design tool. Movement code written here is meant to be portable to the real robot — keep the kinematics layer clean and free of rendering concerns.

## Tech stack

- **Python 3** — single language, no build step.
- **pygame** preferred for the UI (sliders, buttons, free-form pen-position picking on the chassis outline). `turtle` is acceptable for a quick first pass but will be outgrown by the slider/button requirements; if you start with `turtle`, plan the migration.
- No package manager / lockfile is in place yet. When adding dependencies, create a `requirements.txt` (or `pyproject.toml`) at the same time — don't leave imports unpinned.

## Robot model — the part that matters most

The chassis is a rectangle parameterized by `width`, `length`, and `wheel_offset` (position of the wheel pair along one side). The two wheels are **directly opposite** each other on the two long sides at that offset. The pen is at an arbitrary point on the chassis outline.

**Real robot specs (the physical build).** Hardcoded as defaults in `sim.py`:
- length 23.2 cm, width 20.4 cm
- wheels 14.4 cm from front (so `wheel_offset` from back = 8.8 cm)
- wheel diameter 6.6 cm

The slider labelled "Wheels from front" matches how the spec sheet talks about the robot; internally the geometry is still parameterised by `wheel_offset` from the back. The conversion lives in `sim.build_geometry`.

This is a **differential-drive** robot. The movement core must implement differential-drive kinematics:

- Inputs are the two wheel velocities `v_left`, `v_right`.
- The robot's instantaneous center of rotation lies on the wheel axis.
  - `v_left == v_right` → straight line.
  - `v_left == -v_right` → rotation in place about the midpoint of the wheel axis.
  - Otherwise → circular arc; radius `R = (L/2) * (v_l + v_r) / (v_r - v_l)` where `L` is the wheelbase (distance between the two wheels = chassis width).
- The robot's pose `(x, y, θ)` is the pose of the **wheel-axis midpoint**, not the chassis center. These only coincide when `wheel_offset` puts the wheels at the chassis midline.
- The **pen position** is a fixed offset in the chassis frame. To get the world-space pen trace, transform that offset by the current pose each tick. The drawn curve is the pen's path, not the robot's path — this is the whole point of the simulator: a pen mounted off-axis traces a different (and often more interesting / more constrained) curve than the robot body follows.

A clean separation to maintain:

1. **Kinematics layer** — pure functions / a small class that, given `(v_left, v_right, dt)` and current pose, returns the new pose. No pygame imports here.
2. **Path/command layer** — primitives like `move_straight(distance)`, `rotate_in_place(angle)`, `arc(radius, angle)`, plus a way to compose them. This is what gets ported to the real robot.
3. **Simulation/render layer** — owns the pygame loop, draws chassis + wheels + pen marker + accumulated pen trace, hosts sliders/buttons.

## UI requirements (from the project brief)

- Render: chassis outline, both wheels, pen marker, and the accumulated drawn path.
- **Three sliders**: chassis width, chassis length, wheel offset along the side.
- **Pen position picker**: the pen must lie on the chassis outline. Reasonable approaches to evaluate:
  - Click-on-outline: user clicks anywhere near the rectangle perimeter and the point snaps to the nearest edge.
  - Single "perimeter parameter" slider `s ∈ [0, perimeter)` that walks the outline — simple, always valid, easy to reason about.
  - Edge dropdown + 0..1 slider for position along that edge.
  Prefer the perimeter-parameter slider for a v1 — it's the smallest UI surface that guarantees the pen stays on the outline as width/length change.
- **Start / Stop / Reset** buttons. Reset clears the drawn trace and returns the robot to its starting pose; Stop pauses the simulation without clearing.

When a slider changes, the chassis geometry should update live; the existing pen trace should be preserved (the user is comparing what's been drawn against geometry tweaks).

## Script DSL

The console accepts one statement per line; `scripts/*.script` files use the same grammar. Loaded via the panel's script cycler, or typed live into the console (Enter appends to the running program).

```
forward <cm>            # straight, negative ok
back <cm>               # sugar for forward -<cm>
turn <deg>              # in-place, + = CCW (left)
left <deg>              # sugar for turn
right <deg>             # sugar for turn -
arc <radius> <deg>      # + angle = left
circle <radius>         # full CCW circle
speed <cm/s>            # set linear speed (default 12)
angular_speed <deg/s>   # set rotation speed (default 180)
# comment
```

Defaults reset per script (top of `parse_script`). Each line maps to exactly one `WheelCommand` (no loops/macros — keep grammar trivial).

## Known issues to fix in the movement layer

- **Command-boundary timestep splitting.** `CommandRunner.advance` consumes `dt` past a command boundary by incrementing `_idx`, but the surrounding loop in `sim.run` calls `step()` once with the *current* command's velocities for the *full* `dt`. When a command ends mid-frame, the leftover time integrates the wrong (v_left, v_right). Fix: have `step` (or the runner) split a `dt` that crosses a boundary into per-command sub-steps. The square-script test currently uses a loose tolerance because of this.
- **Geometry width changed mid-run.** `WheelCommand` velocities are computed from the wheelbase at parse time. If the user moves the Width slider while a turn or arc is executing, the new effective angular velocity is `(v_r - v_l) / width_new` — not what the script asked for. Fix options: (a) re-parse on width change, (b) refactor commands to be wheelbase-independent ("turn 90° at 180 deg/s") and resolve to wheel velocities each step.

## Things to be careful about

- **Frame conventions.** Pygame's y-axis points down; standard robotics math has y-axis up. Pick one convention for the kinematics layer (y-up is conventional) and convert only at render time. Mixing them silently is a top source of "why is my circle going the wrong way" bugs.
- **Time step.** Drive the simulation off a fixed `dt` (e.g. 1/60 s) decoupled from frame rate where possible. Curves that look fine at 60fps can degenerate at 15fps if `dt` is just "time since last frame".
- **Wheel offset semantics.** Decide once whether `wheel_offset` is measured from the chassis center or from one end, document it in the kinematics module, and don't change it. Slider labels must match.

## Commands

- Install dependencies: `pip install -r requirements.txt`
- Run the simulator: `python -m drawingrobot`
- Run tests: `python -m pytest tests/`
- Run a single test: `python -m pytest tests/test_kinematics.py::test_arc_returns_to_start_after_full_circle`

The kinematics, robot, and commands modules have no pygame dependency — tests run without pygame installed. Only `sim.py` and `ui.py` import pygame.
