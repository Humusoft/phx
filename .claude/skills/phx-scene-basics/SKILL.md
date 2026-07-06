---
name: phx-scene-basics
description: >
  Build and run a physics scene with the PHX MATLAB toolbox (phx.* objects over
  the Bullet engine). Use when creating phx.Body objects, attaching phx.shape.*
  geometry, setting pose/mass/type, stepping a phx.Simulation, or running a PHX
  scene headlessly. Start here before the other phx-* skills.
---

# PHX scene basics

PHX is a MATLAB toolbox (package root `phx/+phx`, targets R2026a) — a thin
object-oriented layer over the Bullet physics engine reached through one MEX
gateway (`phx.engine.io`). You build a scene from `phx.*` handle objects that
draw into MATLAB axes via `hgtransform`, then step it with a `phx.Simulation`.

## Path & how to run

The toolbox lives under `phx/`; it must be on the MATLAB path. Demos live in
`examples/` and each `phxex_*` script does its own `addpath('../phx')`, so run
from `examples/`:

```powershell
# Headless: -batch exits MATLAB when done and returns non-zero on error
& "C:\Program Files\MATLAB\R2026a\bin\matlab" -sd "examples" -batch "phxex_minimal1"

# Or put phx on the path yourself
& "C:\Program Files\MATLAB\R2026a\bin\matlab" -batch "addpath('phx'); cd examples; phxex_minimal1"
```

Wrap long runs with a timeout — engine demos can take minutes. Graphics-dependent
scenes still run under `-batch` (invisible figures), so headless verification works.

## The minimal scene (memorize this shape)

```matlab
clf; view(3); axis("equal"); grid("on"); camlight("headlight");

phx.Body("Type", "static");                 % a default ground/box, immovable
phx.Body("Position", [0.6 -0.5 2]);         % a default dynamic body up in the air

sim = phx.Simulation;                        % collects all bodies in the current axes
sim.step(1, 100, 1);                         % advance 1 s in 100 substeps, redraw each
delete(sim);                                 % tear down engine counterparts
```

A typical scene is 5–10 lines. Bodies created with no axes argument attach to `gca`;
pass an explicit axes as the **first** positional argument when you manage your own:
`phx.Body(ax, "Position", ...)`.

## phx.Body — the principal object

Bodies are movable objects. Construct with name-value pairs (any public property is
settable at construction via `Options.?phx.Body`):

```matlab
b = phx.Body(ax, ...
    "Type", "dynamic", ...                   % "static" | "kinematic" | "dynamic"
    "Position", [0 0 2], ...
    "EulerAngles", [0 0 pi/3], ...           % z->y->x order; or Quaternion / AxisAngle / Orientation
    "Mass", 50, "Inertia", [40 40 10], ...   % set explicitly, or let the shape's density compute it
    "Friction", [0.5 0 0], ...               % [drag roll spin], each >= 0 (no upper bound)
    "Shape", {"Box", "Size", [4 3 0.1], "Color", [1 1 1]});
```

- **Type**: `static` (infinite mass, never moves — floors, walls), `kinematic`
  (you drive its pose, it pushes others), `dynamic` (engine-driven).
- **"Hold then release" idiom**: to hang a body still and later let it fall, create
  it `kinematic` and switch it to `dynamic` at the release moment
  (`body.Type = "dynamic"`). Do **not** start it `static` for this: a body switched
  from `static` to `dynamic` mid-run keeps simulating in the engine but its graphics
  stay **frozen** (static bodies are excluded from the redraw pipeline, so
  `updateView` never runs for it until a pipeline rebuild) — the object falls
  invisibly. `kinematic` bodies are in the redraw pipeline, so they render correctly
  through the switch.
- **Pose** is read/written through these *dependent* properties (they hit the engine
  live): `Position`, `Orientation` (3x3), `Quaternion`, `AxisAngle`, `EulerAngles`,
  `Transform` (4x4). Reading `b.Position` mid-simulation returns the current state.
- **State you can read**: `LinearVelocity`, `AngularVelocity`, `TotalForce`,
  `TotalTorque`, `Energy`.
- **Mass/Inertia footgun**: if you set `Mass` but leave `Inertia` unset, the shape
  computes `Inertia` from its **`Density`** (i.e. from the shape's *natural* mass),
  **not** from your overridden `Mass` — so the two silently disagree and the body
  rotates as if it were far heavier/lighter than it is (a thin `Box [8 1 0.1]` at the
  default `Density` 1000 is an 800 kg block → inertia ~4267 about its long axis, even
  if you set `Mass` to 4). To stay consistent either (a) control the mass through the
  shape's `Density` and let `Inertia` follow, or (b) set `Inertia` explicitly together
  with `Mass`. The demos do (b), e.g. `"Mass", 8, "Inertia", 10`.
- Change `b.Type = "static"` etc. at runtime. Adding (`sim.addObjects`) or deleting
  (`delete(obj)`) objects mid-run rebuilds the simulation pipelines automatically —
  no manual rebuild needed (see phx-engine-gotchas).

## Shapes (`phx.shape.*`)

A `Body` wears geometry. The compact form is the `"Shape"` name-value with a cell
array `{ClassName, Name, Value, ...}`:

```matlab
phx.Body("Shape", {"Box",      "Size", [5 3 0.1]});            % Size = [X Y Z]
phx.Body("Shape", {"Sphere",   "Radius", 0.5});               % or "Diameter", 1
phx.Body("Shape", {"Cylinder", "Radius", 0.3, "Height", 2, "Axis", "z"});
phx.Body("Shape", {"Cone",     "Radius", 0.3, "Height", 1});
phx.Body("Shape", {"Capsule",  "Radius", 0.25, "Height", 1});
phx.Body("Shape", {"path/to/model.stl", "Scale", 0.06*[1 1 1], "Material", "shiny"});
```

Available: `Box`, `Sphere`, `Cylinder`, `Cone`, `Capsule`, `Globe`, `Mesh`,
`Extrusion`, `Revolution`, `Rock`, `Terrain`, `OBJ`, `STL`. Sizing properties are
shape-specific — `Box` uses `Size`; round shapes accept **`Radius`** *or* `Diameter`
(interchangeable), and the axial extent is **`Height`** (not `Length`) with a modeling
`Axis` (`"x"`/`"y"`/`"z"`) on `Cylinder`/`Cone`/`Capsule`. Most accept `Color` and
`Density` (auto-computes mass/inertia if you don't set them), and texture/material
options. `SkeletPoints` adds attachment points used by joints/springs:
`{"Box", "SkeletPoints", [1.5 0 0; -1.5 0 0]}`.

## phx.Simulation — lifecycle

```matlab
sim = phx.Simulation;                 % all bodies in gca (+ their children)
sim = phx.Simulation(bodies);         % a Body array / cell array (children auto-included)
sim = phx.Simulation(ax);             % a specific axes
sim = phx.Simulation("scene.mat");    % bodies saved in a MAT file

sim.Gravity = [0 0 -9.81];            % default; settable
sim.step(interval, substeps, redrawStep);
sim.addObjects(moreBodies);           % rebuilds pipelines
delete(sim);
```

`step(interval, substeps, redrawStep)`:
- `interval` — seconds of simulated time advanced by this call.
- `substeps` — number of physics substeps; the engine timestep is `dt = interval/substeps`.
- `redrawStep` — redraw every Nth substep (use `1` for smooth animation, a larger
  number to skip frames and run faster).

Run it repeatedly to interleave control logic between steps (read body state, set an
actuator, step again) — see the **phx-constraints-forces** skill for the control-loop
pattern. Keep `dt` small (≤ 5 ms) for tight constraint networks or they go unstable.

## Practical tips (physical tuning)

The skills cover the API; the physics still needs sane numbers. Common pitfalls when
building a scene from scratch:

- **Placing bodies that rest on each other.** There is no "place A on top of B" helper
  — compute the resting pose from shape half-extents (a `Box` of height `h` sitting on
  a surface whose top is at `z_s` has its centre at `z_s + h/2`; a `Sphere` of radius
  `r` rests with its centre at `z_s + r`). If exact contact is fiddly, place the body
  a little above and run a short **settle phase** (a few `step` calls) before the main
  action, letting it drop into contact.
- **Friction `[drag roll spin]`, each `>= 0` (no upper bound).** `drag` is the sliding/contact
  friction (≈0.5–0.8 for grippy contact, 0 for near-frictionless; values above 1 are allowed for
  very grippy contact). `roll`/`spin` resist rolling
  and spinning of rounded shapes; leave them at 0 and a ball or sphere will roll/spin
  forever. Give them a small value (≈0.05–0.2) when you need rolling or spin energy to
  dissipate (e.g. bodies that should come to rest instead of ringing).
- **Tuning a launch / lever / impact.** Nothing predicts the outcome for you. For a
  seesaw-style launch, make the dropped mass clearly heavier than the launched body
  and give it a real drop height, then **verify by logging the launched body's apex**
  (`max(body.Position(3))` across the run) and iterate. Keep `Inertia` consistent with
  `Mass` (see the footgun above) or the lever will feel wrong even with the right masses.
- **Stability.** Keep `dt = interval/substeps ≤ 5 ms` for contact-heavy or tightly
  constrained scenes; raise `substeps` rather than taking bigger steps.

## How the object model works (only when subclassing)

Every simulable object subclasses `phx.base.Object`, which holds a DAG (`Parents`/
`Children`), an `hgtransform` (`Graphics`) and an engine handle (`ObjectHandle`).
Subclasses implement four methods: `initObject(obj, world)` (build/refresh engine
counterpart; return `false` to exclude), `destroyObject(obj)`, and the **static,
batch-by-class** `resolveState(cellObjs, dt, time, world)` and
`updateView(cellObjs, dt, time, world)`. New objects route **all** engine access
through `phx.engine.io` — never call Bullet directly. Use the `phx.template.Interaction`
class as a starting point.

## Related skills

- **phx-constraints-forces** — joints, springs, ropes, thrusters, fields, scripted/closed-loop control.
- **phx-logging-view** — Logger, Trace, Measure, Camera, the interactive Viewer, plotting.
- **phx-engine-gotchas** — the `phx.engine.io` gateway, engine variants, error IDs, known quirks, tests.
- **phx-simulink** — drive a PHX scene from Simulink via the PhxModel co-simulation block.
