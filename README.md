# Robot Gait Lab

An interactive robotics playground for freshman engineering students: build robots with 2- or 3-DOF legs, explore walking gaits, and program motion with Arduino-style sketches.

- **Robot builder and gait lab:** open `index.html` to design a body, attach legs, and coordinate walking.
- **Single-leg lab:** open `single-leg.html` to explore joint motion, 3D foot trajectories, and orthographic projections.

Both modules run offline in a modern browser, with no installation, external dependencies, or build step.

Repository: [purwar-lab/robot-gait-lab](https://github.com/purwar-lab/robot-gait-lab). GitHub Pages: [Robot Gait Lab](https://purwar-lab.github.io/robot-gait-lab/). The separate [two-DOF simulator](https://purwar-lab.github.io/two-dof-leg/) is maintained independently.

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


## Robot builder and gait lab

Open `index.html` to arrange 2?8 mounts on a rectangular or circular plate. Attach or detach individual legs and choose 2 or 3 DOF for each. The existing `single-leg.html` single-leg lab remains available.

- Start with a hexapod or quadruped preset, or customize the plate dimensions and leg count.
- Select a leg by clicking its hip or its mount card. Use the joint sliders or the one-leg Arduino example to test it with the chassis held still.
- Choose a gait, stride, foot lift, and cycle time. Gaits include wave, tripod/ripple for six legs, trot/pace/bound for four, and alternating groups for other even counts.
- Walk over the ground, pause, scrub a cycle, and inspect stance/swing phase tracks, foot trails, and the support polygon. Unreachable trajectories stop the Walk action and identify the problem.
- Switch the sketch scope to Whole robot and generate a gait sketch. Every attached joint gets a named servo; 2-DOF legs omit the lateral motor. Check, run, or export the `.ino` file. Changing the design never overwrites editor text automatically.

### Robot model

World axes and joint signs match the single-leg lab (X sideways, Y down, Z forward). The ground is Y=0; the plate is at Y=?height. Links remain 140 model units. Mounts lie around the plate edge, with forward-swing axes aligned in the world Z direction. A 2-DOF leg uses a fixed 18-degree outward lateral bracket; a 3-DOF leg controls that lateral angle. The model does not add an independent coxa link or arbitrary mount yaw.

In gait mode, inverse kinematics solves foot targets within 0?180-degree servo limits. Stance feet move backward relative to the body at exactly the body's forward speed, so their world positions stay fixed. Swing feet follow a smooth lifted path. Disabling Travel over ground holds the body still for inspection and therefore lets feet slide over the ground.

One-leg sketches hold the chassis fixed. Whole-robot sketches estimate horizontal body translation from the incremental motion of at least two retained ground contacts. Contact entry/release use separate tolerances (0.5/0.9 units for all-pulse sketches, 3/5 units for degree or mixed sketches) to avoid contact chatter. The body is not pulled back toward conflicting historical anchors when contacts change. This remains approximate; contact disagreement is displayed as slip. Setup positions the joints with the chassis fixed; estimated travel begins in loop(). Penetration is shown, not resolved. The ideal gait preview is the exact planted-foot reference.

The shaded polygon connects stance feet, and its center marker is the body center projected onto the floor. This is a geometric support demonstration, not a dynamic stability verdict. There is no simulation of gravity, tipping, friction, torque, collision response, or actual center of mass. All exported pin assignments are placeholders and require hardware-specific calibration.

### Prototype files and tests

`robot-core.js` contains pure kinematics, gait phases, support geometry, and sketch generation. `robot-arduino.js` adapts the existing restricted parser to named joints across multiple legs. `robot-motion.js` provides continuous sample interpolation and incremental contact estimation. `robot-app.js` connects the UI, camera, Canvas 3D scene, and code playback. These are ordinary browser scripts with no external dependencies or build system.

Run `node tests/robot.test.cjs` for the robot-model regressions, `node tests/robot-playback.test.cjs` for code-playback precision/contact/timing regressions, and `node tests/simulator.test.cjs` for the original single-leg regressions. Open `tests/robot-browser.html` through a local server for robot UI smoke checks.


### Generated code precision and smooth playback

Generate gait sketch now defaults to **Fine pulses**: 32 samples per cycle, exported using `Servo.writeMicroseconds()`, with the corresponding modeled angle in a comment. The robot parser uses the [Arduino Servo library's default 544?2400 microsecond range](https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h), mapped linearly to 0?180 degrees. A one-microsecond step is about 0.097 degrees in this model; `Servo.write()` uses whole-degree commands. Custom `attach(pin, min, max)` calibration is not supported by this restricted parser and is rejected rather than silently misinterpreted. Hardware pulse-to-angle calibration and servo response remain outside the model.

The Generated commands selector retains **Whole degrees** for beginner sketches. Existing degree-based code still runs, but its quantized targets can yield visibly coarser foot paths. Refresh the lab and regenerate a sketch to adopt fine pulses; existing editor text is never silently replaced.

Robot code playback linearly interpolates joint samples across their delay intervals, without restarting an ease-in/ease-out curve at each sample. Loop boundaries preserve elapsed time, and integer delays sum to the requested cycle duration exactly. The preview follows the actual parsed commands; it does not substitute the predefined gait or snap drawn feet to the floor. Consequently it remains an approximation to the ideal analytic gait, even at fine resolution. The standalone single-leg lab retains its original educational playback model and supported command subset.

## Scene and joint controls

The single-leg lab shows an open rectangular chassis frame, shaded links, and the same world-coordinate foot path in all views. Pose sliders sit beside the canvas on desktop and directly beneath it on narrow screens. The robot lab also places Test one leg inside the scene workspace so joint changes can be watched without scrolling between sections. The chassis frame is a visual mount reference; it does not change kinematics or enforce collisions.

The site homepage opens the full robot builder and gait simulator. Use its Single-leg lab link to open `single-leg.html`. The previous `robot.html` address redirects to the homepage.

Joint axes can be toggled in the single-leg camera toolbar. Gold marks the lateral axis, teal hip flexion, and purple the knee. Flexion and knee axes follow the proximal lateral rotation. In the gait lab, Body construction selects a thin plate or a raised box (drum for a circular outline); Transparent body toggles translucency. Body construction is visual and leaves hip mount locations and walking kinematics unchanged.
