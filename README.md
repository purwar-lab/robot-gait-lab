# 3-DOF Leg Arduino Motion Playground

A standalone, offline simulator for freshman engineering students. Open `index.html` in a modern browser; no installation, external dependencies, or build step is needed.

## Using the playground

- Adjust hip flexion/extension, hip lateral, and knee servo commands (0–180°, neutral 90°).
- Orbit by dragging, pan with Shift/right-drag, and zoom with the wheel. Touch supports one-finger orbit and two-finger pan/pinch. Use the XY/XZ/YZ presets or reset the camera.
- Inspect the simultaneous projections and the orange foot path. Clear the path to start a new experiment. Hidden paths are not recorded; storage is capped at 2,000 distinct samples.
- Place the editor cursor, then insert one servo or all three plus a delay. Without a selected insertion position, commands go at the end of `loop()`.
- Build to validate the sketch and inspect its timeline. Play runs setup once and repeats loop if enabled. Pause, stop, step, or click a timeline row to inspect poses.
- Copy the sketch or download `Three_DOF_Leg_Custom.ino`. No commands are sent to hardware.

Manual pose changes, sketch edits, insertion, and joint-mode changes stop playback. Camera controls remain usable during playback. The compact angle box keeps numerical labels out of the moving geometry.

## Coordinates and joint conventions

The hip is at the origin of a right-handed world: **X sideways, Y downward, Z forward**. Both links are 140 model units; neutral places the knee at `(0,140,0)` and foot at `(0,280,0)`. Units are illustrative, not a calibrated physical length.

With servo commands `h = hip`, `s = hipSide`, and `k = knee`, define angles in degrees:

```text
a = h - 90       # positive hip flexion toward +Z at neutral lateral angle
b = 90 - k       # knee below 90 bends the shin toward +Z
l = -(s - 90)    # lateral servo mounting sign
R = Rz(l)       # abduction/adduction; s above 90 moves a hanging leg toward +X
R = Ry(l)       # alternative yaw mode, around the downward vertical axis

knee = R * Rx(a) * [0,140,0]
foot = knee + R * Rx(a+b) * [0,140,0]
```

The lateral joint is proximal to the flexion joint. Hip flexion and knee bend about the lateral-rotated local X axis; the knee rotation is relative to the thigh. Yaw has no effect on a fully straight vertical leg. A positive yaw servo offset turns a forward-displaced foot toward negative X.

Every view and the recorded trajectory use `computeLeg()` world coordinates. Main presets and small panels use the same orientations: XY has X right/Y down; XZ has X right/Z up; YZ has Z right/Y down. The fixed panel scale accommodates the full 280-unit reach. Camera changes never alter stored path coordinates.

Holding hip and knee fixed while changing abduction produces a circle in XY with constant Z; its XZ projection is a straight segment. Yaw sweeps a circle in XZ only when the foot is away from the Y axis. Combined joint motion need not trace a circle. Only sampled motion is drawn; no synthetic guide arcs are added.

## Sketch subset and playback

Supported: `Servo` declarations with recognizable hip, hipSide/abd/yaw, and knee names; `attach(pin)`; `write(expression)`; `delay(expression)`; initialized `int`, `long`, `unsigned int`, `float`, `double` and `const` variables; assignments; parentheses; unary signs; and `+ - * /`. Includes and `Serial.begin(number)` are ignored. Use one declared servo per joint. Pin attachment is informational; names determine joint roles.

Setup writes and delays appear in the timeline and play once. Global variables and servo targets persist across loops; setup and loop local variables have separate scopes, and loop locals are recreated each iteration. Integer division and integer assignment truncate toward zero. Unsupported commands, missing semicolons, undeclared names, constant assignments, and out-of-range values report errors with source lines. A later loop can stop with an error if an evolving variable exceeds the supported range.

The sketch is parsed as data, never evaluated as JavaScript. This is not an Arduino compiler: control flow, custom functions, sensors, multiple declarations in one statement, microsecond servo writes, integer overflow, and hardware timing are not simulated. Sketches are limited to 1,000 statements and delays to 120,000 ms.

During each delay, targets are interpolated over at most 230 ms of simulated time. Consecutive writes before a delay move together; trailing writes are applied when the iteration ends. Speed scales this educational animation. A repeated loop with no delay is limited to one iteration per animation frame. Physics, torque, collisions, and foot-ground contact are not enforced.

## Local checks

Run `node tests/simulator.test.cjs` for kinematics, projection orientation/framing, parser, playback, trajectory, insertion, and export regression checks. Node is needed only for these development tests, not for the simulator.

With the local server running, open `http://127.0.0.1:8000/tests/browser.html` for browser smoke checks against the actual page, including narrow-layout overflow. The tests change only the embedded page's in-memory state.

For local browser testing, open `index.html` directly or serve this folder with `python -m http.server 8000 --bind 127.0.0.1` and visit `http://127.0.0.1:8000`. The same file can be hosted on a static site such as GitHub Pages.
