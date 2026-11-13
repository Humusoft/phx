---
name: phx-simulink
description: >
  Drive a PHX physics scene from Simulink through the PhxModel block
  (PhxLibrary.slx), with its helpers PHX Action (events) and PHX Point Forces
  (forces at several points of a body). Use when wiring Simulink signals to phx.* object properties
  (input/output port references like Body.Position(1:3)), setting the block's
  scene Source / sample time / viewer, building or debugging a PHX co-simulation
  model, or doing closed-loop control of a PHX scene from Simulink.
---

# PHX in Simulink

PHX integrates with Simulink as **one co-simulation block**: the whole scene is
authored in MATLAB/`.mat` (outside Simulink), the block embeds a `phx.Simulation`
and steps it once per sample time, and Simulink exchanges a few signals with it at
the boundary. The control philosophy is **property-driven**: input/output ports are
bound to *public properties* of named scene objects.

- Block library: **`phx/PhxLibrary.slx`** → block **"PHX Model"** (a Level-2
  MATLAB S-function, `phx/PhxModel.m`, backed by `phx.simulink.BlockBackend`); next to
  it **"PHX Action"** and **"PHX Point Forces"** (both below).
- Worked examples in `examples/`: **`phxex_sim_stand.slx`** (open-loop, sine-driven
  suspension stand), **`phxex_sim_ballplate2.slx`** (closed-loop centroid control of two
  balls in a tilting bowl), **`phxex_sim_sorter.slx`** (vision-in-the-loop color
  sorter: rendered camera → Computer Vision Toolbox → trapdoor; a PHX Action block also
  spawns the cubes onto the carousel at runtime), and **`phxex_sim_swingover.slx`**
  (a scheduled trolley path swings a load over a wall). Their scenes are built by
  `model_*.m` → `saved_*.mat`.

## Block parameters (the mask)

| Mask prompt | param name | meaning |
|---|---|---|
| Source | `ModelSource` | `.m` script or `.mat` file building the scene (Browse button provided) |
| Input ports | `InputRefs` | newline/comma list of references *written* each step |
| Output ports | `OutputRefs` | references *read* each step |
| Show viewer | `ShowViewer` | open an interactive 3-D `phx.extra.Viewer` |
| Redraw steps | `RedrawSteps` | redraw every N-th sample time (default 1); 0 = update the view without a redraw, −1 = leave it alone. Same meaning as `redrawStep` of `phx.Simulation.step`, each sample time being one step |
| Camera position / target | `CameraPosition` / `CameraTarget` | initial view |
| Camera resolution [H W] | `CameraResolution` | `[0 0]` off; `[H W]` adds a rendered-image output port |
| Simulation step | `SimulationStep` | block sample time (s) = the engine step; one engine step per sample |
| Simplified icon | `SimpleIcon` | show only the source on the icon, or label every port |
| *Edit ports…* (button) | — | launches `phx.simulink.portEditor` to pick references |

There are no substeps: each sample time is exactly one engine step. A slower controller
gets its own sample time (Simulink multirate, Rate Transition where needed), and the
viewer's cost is cut with `RedrawSteps`, not with a longer block step. A redraw costs
more than a physics step: at `SimulationStep = 0.001` redraw every ~20th sample.

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
blocks). The frame is `getframe` of the viewer axes, resized to `[H W]` with `imresize`
(base MATLAB, no toolbox needed). Aim the camera with `CameraPosition`/`CameraTarget`
(e.g. top-down `[0 -1 25]` → `[0 0 0]`). Notes: it requires the viewer (`ShowViewer`
on), updates the view every sample whatever `RedrawSteps` is (each frame costs a render —
sample the vision loop coarsely), and works headless under `-batch`. The block enables N-D signals (`AllowSignalsWithMoreThan2D`)
for this port. Ground-truth object properties remain available on the other ports for
validating the vision estimate.

## What to use as inputs / outputs (property-driven control)

Because ports are properties, choose them semantically:

- **Inputs** = *settable* properties. The actuators are properties already:
  - **Force / torque on a body**: its `AppliedForce` (N, at the centre of mass) and
    `AppliedTorque` (N·m), both in the body frame, held until the next sample — e.g.
    `Drone.AppliedForce`, or `Bob.AppliedForce(3)` for a push along the body z axis.
    Forces at several points go through the **PHX Point Forces** block (below). A
    `phx.Thruster`'s `Throttle` (−1…1, scaled by `MaxThrust`) only when you want its
    lag, reaction torque or drawn exhaust.
  - **Joint motors**: `TargetVelocity` of a `RevoluteJoint` / `PrismaticJoint`, with
    `MaxTorque` / `MaxForce` as the motor's strength (also settable from a port).
  - **Joint torque / force**: `AppliedTorque` of a `RevoluteJoint`, `AppliedForce` of a
    `PrismaticJoint` — a pure torque/force between the two bodies, held until the next
    sample. The input for actuators computed elsewhere, e.g. a Simscape cylinder whose
    force drives the joint. It acts together with the motor, which then regulates
    against it like against any other load.
  - **Prescribed motion**: `Position` / `EulerAngles` / `Orientation` of a `kinematic`
    body (see the kinematic note below), or `LinearVelocity` / `AngularVelocity`.
  - Parameters such as `Mass`, `Friction` or a spring's `Stiffness`.

  Only the methods (`applyForce`, `applyTorque`) are not ports; call them from a PHX
  Action block when an event needs them.
- **Outputs** = any readable property: `Position`, `Orientation`, `LinearVelocity`,
  `TotalForce`, `Energy`, a `Spring.Length`, a joint angle, etc. (read-only ones like
  `TotalForce`/`Energy` are output-only).

## Co-simulation execution & feedback loops

Per sample time the block does **read outputs** (`Outputs`), then **write inputs → step**
(`Update`). The state advance lives in `Update` because Simulink calls it exactly once per
major time step, while `Outputs` may be called more often and must stay side-effect free.

Consequences: the ports carry the scene state **at the current time** (at `t = 0` the
initial scene, before any physics), and an input arriving at `t` drives the step from `t`
to `t+dt`, so its effect shows up on the outputs one sample later — a built-in one-step
delay.

**Closing a feedback loop** (output → controller → input): wire it directly. Because the
inputs are consumed in `Update` and never read in `Outputs`, the block declares
`DirectFeedthrough = false`, which breaks the loop by itself — **no Memory or Unit Delay
block is needed**, and adding one only piles a second sample of delay onto the built-in
one. `phxex_sim_ballplate2` and `phxex_sim_sorter` close their loops this way.

(Before this, the block declared direct feedthrough and a plain feedback wiring raised
`Simulink:Engine:AlgLoopTroubleInFixedStepNoSolverSwitch`; the algebraic-loop solver also
re-entered `Outputs` — and therefore `Sim.step` — several times per sample time.)

## KINEMATIC control (the key gotcha)

A pose written to a kinematic body is reached in the one engine step of the sample, so
the body moves by the whole per-sample increment at once. **Use a small `SimulationStep`
for pose-driven scenes** (the ball-in-bowl demo needs `dt = 5 ms`): large increments make
the body punch away whatever it touches (explosive ejections).

Related: **never set a kinematic pose in one large jump** even outside Simulink —
teleporting a kinematic body into a resting body ejects it. Ramp it.

## PHX Action block (events / structural changes)

The main block is declarative (property ports). For **occasional imperative work** —
adding/removing bodies at runtime, one-off method calls (`applyForce`, reparenting,
any `phxEngine`) — use the **PHX Action** block (`PhxLibrary`, `PhxAction.m`). On a
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

## PHX Point Forces block (forces at several points)

A body takes one force at its centre of mass and one torque. The **PHX Point Forces**
block (`PhxLibrary`) turns N forces at fixed points into exactly that:
`F = ΣFk`, `M = Σ(Pk − Origin) × Fk (+ T)`. Mask params: `Points` (N×3, one row per
force; sets the number of inputs), `Origin` (the reference point of `M`), `TorqueInput`
(adds a pure torque input `T`). Inputs `F1..FN` (3-vectors), outputs `F`, `M`. It does
no rotation: give forces and points in the **body frame**, `Origin` = the body's
`CenterOfMass` (`[0 0 0]` for box, sphere, cylinder, capsule), and wire `F` → `Name.AppliedForce`, `M` →
`Name.AppliedTorque` of the PHX Model block. Plain Simulink inside (Mux + two matrix
Gains, set up by `phx.simulink.pointForces` in the mask init), so it is a direct
feedthrough and works without PHX too.

```matlab
add_block('PhxLibrary/PHX Point Forces', [M '/Rotors']);
set_param([M '/Rotors'], 'Points', '[0.3 0.3 0; -0.3 0.3 0; -0.3 -0.3 0; 0.3 -0.3 0]', ...
    'Origin', '[0 0 0]', 'TorqueInput', 'on');   % T = propeller reaction torques
% PHX Model InputRefs: sprintf('Drone.AppliedForce\nDrone.AppliedTorque')
add_line(M, 'Rotors/1', 'PHX/1');  add_line(M, 'Rotors/2', 'PHX/2');
```

## Building a PHX co-simulation model programmatically

```matlab
M = 'my_phx_model';
new_system(M); load_system('PhxLibrary');
add_block('PhxLibrary/PHX Model', [M '/PHX']);
set_param([M '/PHX'], ...
    'ModelSource', 'saved_scene.mat', ...
    'InputRefs',   'Plate.EulerAngles(1:2)', ...
    'OutputRefs',  sprintf('Ball1.Position(1:2)\nBall2.Position(1:2)'), ...
    'ShowViewer',  'on', 'RedrawSteps', '4', 'SimulationStep', '0.005');
% ... add controller blocks, wire output -> controller -> PHX input (no delay block) ...
set_param(M, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.005', 'StopTime', '20');
save_system(M);
```

The block has its own discrete sample time (`SimulationStep`) and runs with any solver:
`phxex_sim_stand` uses a variable-step one. With a fixed-step solver, `FixedStep` must
divide the block's `SimulationStep` (equal, or e.g. 0.005 for a 0.01 block step as in
`phxex_sim_ballplate2`); a fixed-step discrete solver suits an all-discrete model. Run headless with
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

- **Kinematic control → small sample time** (above); the sample time is the engine step.
- **Small sample time → raise `RedrawSteps`**: redrawing at every millisecond sample costs
  several times the physics.
- **Feedback loop → wire it directly**, no Memory/Unit Delay: the block is not a direct
  feedthrough and already carries one sample of delay. A delay block adds a second one.
- **Reference an object by `Name`** — set unique `Name`s in the scene builder; an unset/
  duplicate name can't be addressed.
- **Underactuation is real**: a 2-DOF tilt cannot independently place two balls (only
  their centroid is controllable) — pick controllable objectives. See `phxex_sim_ballplate2`.

## Related skills

- **phx-scene-basics** — build the scene (`phx.Body`, shapes, `Name`, `Type`) the block loads,
  and save it to the `.mat` the `ModelSource` points at.
- **phx-constraints-forces** — body loads, joints/springs/thrusters and the in-MATLAB control-loop analogue.
- **phx-engine-gotchas** — `phxEngine`, pipeline rebuilds, `dt` stability, error IDs, tests.
