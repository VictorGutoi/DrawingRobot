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
goto <x> <y>            # drive pen to world point (x, y) along a straight
                        # line; plans rotate+forward, lands pen on target
line_to <x> <y>         # draw a straight pen line from current pen pos to
                        # (x, y); plans rotate-translate-rotate setup + forward,
                        # so each edge of the trace is exactly the polyline edge
trace x1 y1 x2 y2 ...   # track a polyline pen path via feedback linearisation;
                        # pen follows polyline exactly within timestep
                        # discretisation, body weaves underneath; requires px≠0
speed <cm/s>            # set linear speed (default 12)
angular_speed <deg/s>   # set rotation speed (default 180)
# comment
```

Defaults reset per script (top of `parse_script`). Most lines map to one `WheelCommand`; `goto` plans rotate+forward (two commands) and tracks parse-time pose so subsequent gotos build on the previous endpoint.

Two pen-aware path commands:

- **`goto X Y`** — pen *lands* on target. Plans rotate + forward; the rotation arc bulges between targets (centered on the wheel midpoint, not on the polyline corner), so a polyline of gotos visits each corner but the inter-corner pen path is noticeably not the polyline.
- **`line_to X Y`** — pen *draws* a straight line from its current world position to target. Plans rotate-translate-rotate setup + forward; the setup repositions the wheel midpoint so the pen sits at the same world point but with body aligned to the new edge direction. The forward leg traces the polyline edge exactly. The corner curve (the setup pen path) is localized at the vertex.

Setup translation `Δ = (R_θ_curr − R_θ_new) · pen_body` depends only on the heading change, not on the polyline geometry. For a 90° corner with `|pen_body| = 13.5 cm`, `|Δ| ≈ 19 cm` — the corner is geometrically chunky but stays *at the corner* instead of distorting the edges.

Lower bound on corner radius (for **single-arc** corners): with the pen anywhere on the chassis outline except at one of the two wheel positions, a single arc command paints a pen circle of radius ≥ `|px|` (the body-x component of the pen offset). In-place rotation specifically gives radius `|pen_body|`. Hard kinematic floor for any constant-(v, ω) command.

The `trace` primitive sidesteps that floor by changing (v, ω) every timestep — feedback linearisation at the offset pen point, det J = px. Inverting J each step gives wheel velocities that produce *any* commanded pen velocity, so the pen tracks an arbitrary polyline (sharp corners and all) within timestep discretisation. The body trajectory underneath is generally non-obvious — pen leads, body lags. Singular at px = 0; for pen on the wheel-axis line, use line_to/goto instead.

Folklore consequence of the b=0 singularity in De Luca/Oriolo I/O linearisation; the geometric corner-radius lemma isn't published under a name.

## Movement-layer integration

`CommandRunner.consume(dt)` returns `(v_left, v_right, sub_dt)` segments that split `dt` at command boundaries — call `step()` once per segment so velocities switch exactly when a command ends. Don't integrate with `advance()` + `current_velocities()`; that pattern silently runs leftover `dt` against the previous command, which compounds into visibly wrong angles when the pen is off-axis.

## Velocity limits and ROS2 publishing

`drawingrobot.limits.Limits(max_linear_cm_s, max_angular_rad_s)` enforces ceilings on the body's `(v, ω)`. `clamp_vw(v, ω)` scales **both** by the same ratio when either exceeds its ceiling — instantaneous curvature `R = v/ω` is preserved, the body still follows the same arc, just slower.

**Limits apply only to the ROS publish, not to the simulator integration.** The simulator preview always shows what the script asked for; clamping is a hardware safety net for the real robot consuming `/cmd_vel`. This split matters because the `trace` primitive demands peak `|ω|` proportional to `1/p_x` and to corner sharpness — for a 90° corner with the codebase's defaults that's ~60π rad/s for one tick. Clamping the integration would scale `v` down by the same ratio (curvature-preserving), and the resulting visible trace would have nothing to do with the script. With the new split, the preview is honest about the planner's intent and the published Twist is the truncated version the robot actually executes.

The on-robot consequence: scripts that exceed limits produce a correctly-shaped pen path on the simulator but a distorted one on the real robot — typically rounded corners where the planner asked for sharp ones. Diagnose this by comparing the simulator trace against the actual drawing; if they differ, either lower script speeds (`speed`, `angular_speed`) or raise the CLI ceilings.

`drawingrobot.ros_publisher.RosPublisher(topic, node_name)` publishes `(v, ω)` as `geometry_msgs/Twist` (linear.x = v converted to m/s, angular.z = ω in rad/s). The module imports `rclpy` and `geometry_msgs` lazily inside `__init__` so the rest of the codebase stays usable on machines without ROS2; calling `RosPublisher()` without a ROS2 install raises a `RuntimeError` with a clear message. The publisher is created in `sim.run()` only when `--ros` is passed; teardown on QUIT publishes a final `(0, 0)` Twist before `close()`.

CLI:

```
python -m drawingrobot                                      # sim only, default limits 0.5 m/s and 0.5 rad/s
python -m drawingrobot --ros                                # also publish to /cmd_vel
python -m drawingrobot --ros --ros-topic /robot/cmd_vel
python -m drawingrobot --max-linear 0.3 --max-angular 1.0   # tighter linear, looser angular
python -m drawingrobot --headless --script square_trace --ros   # no UI; useful inside a container
```

Limits are passed in **m/s and rad/s** at the CLI (matching ROS conventions); `Limits` stores them in cm/s and rad/s internally.

### Running with ROS2 against the real robot

```
./scripts/bringup_mac.sh                          # default: open the GUI sim, /cmd_vel
./scripts/bringup_mac.sh --script square_trace    # forwards args to drawingrobot
./scripts/bringup_mac.sh --headless --script line_to
```

The Pi runs its own docker-compose stack at `~/Documentos/Luloc-ASTI-25-26/02-software/` with `micro-ros-agent` (UDP4 on port 8888, default RMW = FastDDS) plus lidar / telegraf / grafana / influxdb / mosquitto / ptp_master. The agent stays up under `restart: unless-stopped`; the ESP32-P4 reaches it over Ethernet, not USB. **`bringup_mac.sh` does not manage anything on the Pi** — it only configures the Mac and runs the sim. If the compose stack is down, ssh in and `cd ~/Documentos/Luloc-ASTI-25-26/02-software && docker compose up -d`.

What `bringup_mac.sh` actually does: preflights (`conda env ros2`, `scripts/dds/fastdds-mac.xml` exists, optional ssh probe to `pi5@192.168.50.53` checking that `micro_ros_agent` is up), exports the FastDDS env vars, activates the conda env, and execs `python -m drawingrobot --ros`. Override `PI_HOST=user@host` if your Pi isn't `pi5@192.168.50.53`; set `SKIP_PI_CHECK=1` to skip the ssh probe entirely.

DDS strategy: the Mac speaks FastDDS to match the Pi agent. Multicast discovery actually works on the home network (verified 2026-05-09 by tcpdump on the Pi: Mac multicast announcements reach `wlan0` and the Pi participants reply unicast back to `192.168.50.40:7411`). The Mac's `scripts/dds/fastdds-mac.xml` still pins the transport to `en0` so FastDDS doesn't pick a VPN/utun interface, and includes a unicast `initialPeersList` entry pointing at the Pi as a belt-and-braces fallback for hostile WiFi (corporate AP, hotel). The Pi compose-stack agent needs no XML.

**XML element-order gotcha:** FastDDS parses profiles in document order, so `<transport_descriptors>` must come **before** any `<participant>` that references a custom `<transport_id>`. If a participant references an undefined transport, the parser logs `[XMLPARSER Error] Transport Node not found` and silently rejects the whole profile, falling back to all defaults — which masquerades as "the file isn't being read at all". Both `fastdds-mac.xml` and `fastdds-pi.xml` are ordered correctly; preserve that order on edits.

`scripts/dds/fastdds-pi.xml` is for *diagnostic* containers spawned ad-hoc on the Pi (e.g. `run_pi_listener.sh`). Such a container is its own DDS participant and needs the Mac as an initial peer plus a wlan0 whitelist (the Pi has eth0 on `192.168.5.0/24` for the ESP32-P4 link, so without pinning FastDDS picks the wrong interface). The compose stack does not load this file.

Diagnostic scripts (kept, not deleted):

- `./scripts/run_sim_ros.sh` — Mac sim + ROS publish, no Pi orchestration. Useful to isolate publish-path bugs.
- `./scripts/run_pi_listener.sh` — `ros2 topic echo /cmd_vel` inside a fresh Pi container with FastDDS + the Mac as unicast peer. Useful to confirm cross-host visibility before the agent is in the loop.
- `./scripts/_ros_smoke_publisher.py` — 10 Hz Twist publisher, sanity-checks the publish path without the GUI.

`scripts/bringup_pi.sh` and `scripts/dds/cyclone-{mac,pi}.xml` are legacy from a clean-slate flow that brought up its own serial-attached agent on the Pi with Cyclone DDS. They are unused by the current `bringup_mac.sh` but kept until the FastDDS path is fully verified. If you ever need to run on a Pi without the Luloc compose stack, those files document the alternative path.

Stack gotchas:

1. **The `ros2` CLI is a separate conda package from `rclpy`.** RoboStack's `ros-humble-rclpy` ships only the Python bindings. To run `ros2 topic list` from the Mac, also install `ros-humble-ros2cli` and `ros-humble-ros2topic`.
2. **Mac and Pi must share the same RMW.** The Pi compose stack is FastDDS (image default, no override), so the Mac uses `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `FASTRTPS_DEFAULT_PROFILES_FILE=…/fastdds-mac.xml`. FastDDS↔Cyclone interop over RTPS is technically possible but fragile — pick one. Don't leave `CYCLONEDDS_URI` set in the shell when running FastDDS; bring-up scripts `unset` it but a manually-launched session might inherit it.
3. **`ROS_DOMAIN_ID=0` everywhere.** The Pi compose file leaves `ROS_DOMAIN_ID` unset (default 0). The Mac scripts default to 0. Don't change one without changing the other.

## Known issues to fix in the movement layer

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

The kinematics, robot, and commands modules have no pygame dependency — tests run without pygame installed. Only `sim.py` and `ui.py` import pygame. `ros_publisher.py` imports `rclpy` lazily inside `RosPublisher.__init__`, so the module is importable (and the rest of the package usable) on machines without ROS2 installed.
