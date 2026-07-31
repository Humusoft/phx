---
name: phx-simulink
description: >
  Drive a PHX physics scene from Simulink through the single PhxModel block
  (PhxLibrary.slx). Use when wiring Simulink signals to phx.* object properties
  (input/output port references like Body.Position(1:3)), setting the block's
  scene Source / Substeps / viewer, building or debugging a PHX co-simulation
  model, or doing closed-loop control of a PHX scene from Simulink.
---

# PHX in Simulink

PHX integrates with Simulink as **one co-simulation block**: the whole scene is
authored in MATLAB/`.mat` (outside Simulink), the block embeds a `phx.Simulation`
and steps it once per sample time, and Simulink exchanges a few signals with it at
the boundary. The control philosophy is **property-driven**: input/output ports are
bound to *public properties* of named scene objects.

- Block library: **`phx/PhxLibrary.slx`** → block **"physical simulation"** (a Level-2
  MATLAB S-function, `phx/PhxModel.m`, backed by `phx.simulink.BlockBackend`).
- Worked examples in `examples/`: **`phxex_sim_stand.slx`** (open-loop, sine-driven
  suspension stand), **`phxex_sim_ballplate2.slx`** (closed-loop centroid control of two
  balls in a tilting bowl), and **`phxex_sim_sorter.slx`** (vision-in-the-loop color
  sorter: rendered camera → Computer Vision Toolbox → trapdoor; a PHX Action block also
  spawns the cubes onto the carousel at runtime). Their scenes are built by
  `model_*.m` → `saved_*.mat`.

## Block parameters (the mask)

| Mask prompt | param name | meaning |
|---|---|---|
| Source | `ModelSource` | `.m` script or `.mat` file building the scene (Browse button provided) |
| Input ports | `InputRefs` | newline/comma list of references *written* each step |
| Output ports | `OutputRefs` | references *read* each step |
| Show viewer | `ShowViewer` | open an interactive 3-D `phx.extra.Viewer` |
| Render each step | `RenderEach` | redraw every substep vs. once per sample time |
| Camera position / target | `CameraPosition` / `CameraTarget` | initial view |
| Simulation step | `SimulationStep` | block sample time (s) |
| Substeps | `Substeps` | physics substeps per sample time (**see kinematic note**) |
| Camera resolution [H W] | `CameraResolution` | `[0 0]` off; `[H W]` adds a rendered-image output port |
| *Edit ports…* (button) | — | launches `phx.simulink.portEditor` to pick references |

## Port references — the grammar

A port is described by text resolved against the scene at compile time:

```
ObjectName.Property            % the whole property
ObjectName.Property(indices)   % selected linear indices, e.g. Chassis.Position(1:3)
```

- The object is found by its **`Name`** in the scene (set `Name` on every body you
  reference; names must be unique). One reference per line.
- **Bracket-less = whole property.** `Ball1.Position` expands to `Ball1.Position(1:3)`;
  the signal **width comes from class metadata** (`meta.property` size, e.g. `Position`
  → 3, `Quaternion` → 4, `Transform` → 16), no scene run needed. Add indices only to
  select a subset (`StandFL.Position(3)`).
- **Edit-time validation**: editing `Source`/`Input ports`/`Output ports` validates the
  refs against the scene (`phx.simulink.BlockBackend.validateRefs`) — a wrong object or
  property, or a non-sizeable one, errors immediately with a `phx:PhxModel:*` id instead
  of failing at run time. Validation is best-effort for `.mat` sources (skipped for `.m`,
  to avoid running the builder on every edit).
- **Picker**: the *Edit ports…* button lists the scene's objects and their sizeable
  numeric properties (read-only ones marked, allowed only as outputs) so you assemble
  refs by clicking; leave the Indices field empty (placeholder "all") for the whole property.
- **Methods are not yet supported** as ports (e.g. `applyTorque`). The reference layer
  reserves a `Kind` field for them and errors clearly (`phx:PhxModel:methodPortsUnsupported`);
  drive actuation through *settable properties* instead.

## Rendered-image output (synthetic camera)

Set **`CameraResolution` = `[H W]`** (non-zero, and `Show viewer` on) to add a final
**`uint8 [H W 3]` output port** carrying the rendered view each step — a lightweight
synthetic camera for vision-in-the-loop (feed it to Image Processing / Computer Vision
blocks). The frame is `getframe` of the viewer axes, resized to `[H W]` (needs Image
Processing Toolbox for `imresize`). Aim the camera with `CameraPosition`/`CameraTarget`
(e.g. top-down `[0 -1 25]` → `[0 0 0]`). Notes: it requires the viewer (`ShowViewer`
on), forces a redraw each step (slower — sample the vision loop coarsely), and works
headless under `-batch`. The block enables N-D signals (`AllowSignalsWithMoreThan2D`)
for this port. Ground-truth object properties remain available on the other ports for
validating the vision estimate.

## What to use as inputs / outputs (property-driven control)

Because ports are properties, choose them semantically:

- **Inputs** = *settable* properties. Prescribe motion of a `kinematic` body via
  `Position` / `EulerAngles` / `Orientation`, command `LinearVelocity` /
  `AngularVelocity`, or set `Mass` / `Friction` / `Type`. (Force/torque actuation via
  `applyForce`/`applyTorque` is a method, not yet a port — model the actuator as a
  body whose pose/velocity you command, or as a future `PermanentForce`-style property.)
- **Outputs** = any readable property: `Position`, `Orientation`, `LinearVelocity`,
  `TotalForce`, `Energy`, a `Spring.Length`, a joint angle, etc. (read-only ones like
  `TotalForce`/`Energy` are output-only).

## Co-simulation execution & feedback loops

Per sample time the block does **step → write inputs → read outputs**, so outputs
reflect the just-advanced state and inputs take effect on the *next* step (a built-in
one-step delay that conveniently keeps the block from being an algebraic feedthrough).

**Closing a feedback loop** (output → controller → input): Simulink still treats the
block's input port as direct-feedthrough, so a pure feedback wiring raises an algebraic
loop. Insert a **Memory** (or Unit Delay) block in the loop to break it — this matches
the co-sim semantics and is what the ball-in-bowl demo does.

## Substeps & KINEMATIC control (the key gotcha)

`Substeps > 1` makes the block loop `Sim.step(dt, N)` — N engine steps per sample
time. This is fine for **purely dynamic** scenes (more substeps = smaller engine `dt`
= stiffer-constraint stability).

**But for a pose-driven KINEMATIC body, set `Substeps = 1` and use a small `SimulationStep`
instead.** MATLAB-level substeps are *not* Bullet substeps: the block sets the kinematic
pose **once** per sample time and then loops the engine with the pose frozen, so the
whole per-step pose increment is applied in `dt/N` → the kinematic body looks **N× too
fast** for one substep and violently punches away bodies it touches (explosive ejections).
A small sample time with `Substeps = 1` moves the kinematic body smoothly. (This was the
root cause of the ball-in-bowl "explosions"; `dt = 5 ms`, `Substeps = 1` fixed it.)

Related: **never set a kinematic pose in one large jump** even outside Simulink —
teleporting a kinematic body into a resting body ejects it. Ramp it.

## PHX Action block (events / structural changes)

The main block is declarative (property ports). For **occasional imperative work** —
adding/removing bodies at runtime, one-off method calls (`applyForce`, reparenting,
any `phx.engine.io`) — use the **PHX Action** block (`PhxLibrary`, `PhxAction.m`). On a
rising edge of its trigger (input 1) it runs user MATLAB code against the running scene.
Mask params: `Code`, `NumInputs`, `OutputSizes`, `MainBlock` (the bound scene block,
set via the **Select main block** button → stores its `SID`). The code runs with these
variables in scope: `sim` (the bound `phx.Simulation`), `ax` (viewer axes or `[]`), `t`,
`input1..N`, `state` (persistent struct across triggers — use it to remember spawned
handles, counters, etc.), and assigns `output1..M`. Ports are labeled `trigger` /
`input1..N` / `output1..M`. **Port sizes:** data inputs *inherit* their dimension from the
wired signal; outputs are sized by `OutputSizes`, a cell of dimensions — `{1, 3, [2 4]}`
means output1 scalar, output2 a 3-vector, output3 a 2×4 matrix (count = `numel`). Binding
is resolved lazily at the first step (Start order between blocks isn't guaranteed) and the
scene handle can't be wired (signals are numeric) — hence the SID reference. Keep it for
*events*; continuous actuation belongs on the property bus. Example: `phxex_sim_sorter`
uses it to spawn coloured cubes onto the carousel at regular angular steps during the run.

## Building a PHX co-simulation model programmatically

```matlab
M = 'my_phx_model';
new_system(M); load_system('PhxLibrary');
add_block('PhxLibrary/physical simulation', [M '/PHX']);
set_param([M '/PHX'], ...
    'ModelSource', 'saved_scene.mat', ...
    'InputRefs',   'Plate.EulerAngles(1:2)', ...
    'OutputRefs',  sprintf('Ball1.Position(1:2)\nBall2.Position(1:2)'), ...
    'ShowViewer',  'on', 'Substeps', '1', 'SimulationStep', '0.005');
% ... add controller blocks, wire output -> controller -> Memory -> PHX input ...
set_param(M, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.005', 'StopTime', '20');
save_system(M);
```

The block sets its own `SampleTimes`; give the model a **fixed-step discrete** solver
with `FixedStep` = the block's `SimulationStep`. Run headless with
`-batch "sim('my_phx_model')"` (graphics/viewer work under `-batch`, invisibly).

## `phx.simulink.*` internals (for programmatic / advanced use)

- `BlockBackend.getModelInterface(source)` → struct array `{Name, Class, Properties,
  Methods}` of the scene, engine-free (loads/`feval`s the source, walks the DAG). Basis
  for validation and the picker.
- `BlockBackend.signalProperties(class)` → `{Name, Width, ReadOnly}` of sizeable numeric
  properties of a class. `BlockBackend.propertyWidth(class, prop)` → element count from
  metadata (NaN if not statically sizeable).
- `BlockBackend.validateRefs(source, listText, portKind)` → throws the mask's validation
  errors. `ParameterReference` parses one `Object.Property(idx)` line.

## Gotchas checklist

- **Kinematic control → `Substeps = 1` + small sample time** (above). Substeps>1 only for
  dynamic-only scenes.
- **Feedback loop → insert a Memory/Unit Delay** to break the algebraic loop.
- **Reference an object by `Name`** — set unique `Name`s in the scene builder; an unset/
  duplicate name can't be addressed.
- **Scalar `Friction` broadcasts to `[drag roll spin]`** — `0.6` also sets rolling
  resistance 0.6 and a ball won't roll; use `[0.6 0 0]` (see phx-scene-basics).
- **Underactuation is real**: a 2-DOF tilt cannot independently place two balls (only
  their centroid is controllable) — pick controllable objectives. See `phxex_sim_ballplate2`.

## Related skills

- **phx-scene-basics** — build the scene (`phx.Body`, shapes, `Name`, `Type`) the block loads.
- **phx-constraints-forces** — joints/springs/thrusters and the in-MATLAB control-loop analogue.
- **phx-engine-gotchas** — `phx.engine.io`, pipeline rebuilds, `dt` stability, error IDs, tests.
