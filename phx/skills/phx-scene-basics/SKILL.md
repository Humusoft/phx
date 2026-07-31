---
name: phx-scene-basics
description: >
  Build and run a physics scene with the PHX MATLAB toolbox (phx.* objects over
  the Bullet engine). Use when creating phx.Body objects, attaching phx.shape.*
  geometry, importing meshes (STL/OBJ/PLY) or URDF models, using the phx.assembly
  prefab builders (arena, chain, scatter, import), drawing the scene into
  phx.extra.Viewer (the default) or plain axes, stepping a phx.Simulation, or
  running a PHX scene headlessly. Start here before the other phx-* skills.
---

# PHX scene basics

PHX is a MATLAB toolbox (package root `phx/+phx`, targets R2026a) — a thin
object-oriented layer over the Bullet physics engine reached through one MEX
gateway (`phx.engine.io`). You build a scene from `phx.*` handle objects that
draw into MATLAB axes via `hgtransform`, then step it with a `phx.Simulation`.

## Path & how to run

PHX is used as an **installed toolbox** — `phx.*` is simply available, no `addpath`
is needed and scene code must never contain one. The demos in `examples/` follow
this: a `phxex_*` script is a plain function that assumes PHX is installed.

```powershell
# Headless: -batch exits MATLAB when done and returns non-zero on error
& "C:\Program Files\MATLAB\R2026a\bin\matlab" -sd "examples" -batch "phxex_minimal"
```

Wrap long runs with a timeout — engine demos can take minutes. Graphics-dependent
scenes still run under `-batch` (invisible figures), so headless verification works.

*(Only when working on a PHX source checkout rather than the installed toolbox:
prepend that checkout's `phx/` for the run — `-batch "addpath('phx'); cd examples;
phxex_minimal"` — so the edited sources win over the installed copy. This is a
development detail; never put it into user scene code.)*

## The scene shape (memorize this)

**Default to `phx.extra.Viewer("clear", ...)`** — it is what a PHX scene is meant to be
shown in, and what almost every demo does. It gives orbit/pan/zoom, object picking, a
HUD, sky textures and sane lighting for free; you get back the axes and pass it as the
**first** positional argument to everything you create:

```matlab
[viewer, ax] = phx.extra.Viewer("clear", ...
    "DefaultCameraPosition", [6 -7 3], ...   % where the camera starts (Home returns here)
    "DefaultCameraTarget",  [0 0 0]);

phx.Body(ax, "Type", "static");              % a default ground/box, immovable
phx.Body(ax, "Position", [0.6 -0.5 2]);      % a default dynamic body up in the air

sim = phx.Simulation(ax);                    % collects the objects in those axes
sim.step(1, 100, 1);                         % advance 1 s in 100 substeps, redraw each
delete(sim);                                 % tear down engine counterparts
```

A typical scene is 5–10 lines. The viewer wants the **whole figure** — it is not made
to share a window — so use plain MATLAB axes instead when:

- the scene is a throwaway/minimal illustration (`phxex_minimal`),
- the visualization must live in a **subplot** or an app layout, or several views show
  the same simulation (`phxex_multiview`),
- there is no visualization at all (pass `[]` as the axes — `phxex_noview`).

Plain-axes form: configure `gca` yourself, then let the objects default to it (bodies
created with no axes argument attach to `gca`, `phx.Simulation` with no argument
collects them):

```matlab
clf; view(3); axis("equal"); grid("on"); camlight("headlight");
axis("manual");                              % freeze limits so falling bodies don't rescale

phx.Body("Type", "static");
phx.Body("Position", [0.6 -0.5 2]);

sim = phx.Simulation;
sim.step(1, 100, 1);
delete(sim);
```

Both forms work headless under `-batch` (the viewer included) — headless is *not* a
reason to avoid the viewer; only a `[]` axes really skips the graphics work. Viewer
details (view modes, sky textures, `displayText` HUD, keys) are in **phx-logging-view**.

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
- **Restitution (bounciness)**: `Restitution` (scalar ≥ 0, default **0** = dead contact),
  settable live mid-run. The bounce of a contact **combines the values of both colliding
  bodies** (product-like): a bouncy ball dropped on a default floor does **not** bounce —
  give the floor nonzero `Restitution` too. Realized bounce heights come out noticeably
  below the ideal `e²·h` (solver losses at contact), so tune `e` empirically against the
  bounce you want rather than computing it.
- **Pose** is read/written through these *dependent* properties (they hit the engine
  live): `Position`, `Orientation` (3x3), `Quaternion`, `AxisAngle`, `EulerAngles`,
  `Transform` (4x4). Reading `b.Position` mid-simulation returns the current state.
- **State you can read**: `LinearVelocity`, `AngularVelocity`, `TotalForce`,
  `TotalTorque`, `Energy`.
- **`Mass` and `Inertia` are independent overrides (by design).** A shape computes
  *both* from its `Density` as defaults, but you may override either one on its own —
  e.g. set a custom `Inertia` to model a non-uniform mass distribution, or a custom
  `Mass` without disturbing the shape-derived `Inertia`. Setting one never rewrites the
  other. Practical consequence to keep in mind: if you override `Mass` alone, `Inertia`
  keeps the shape's density-based value (a thin `Box [8 1 0.1]` at default `Density`
  1000 is an 800 kg block → `Inertia_y ≈ 4267`, unchanged even if you set `Mass` to 4),
  so a uniform body spins as if it still had its natural mass. For a *uniform* body of a
  given mass, either drive it through `Density`, or set `Mass` and `Inertia` together —
  the demos do the latter, e.g. `"Mass", 8, "Inertia", 10`.
- Same for the default box: `phx.Body("Mass", 10)` (no explicit `Shape`) weighs 10 kg
  but keeps the unit box's default `Inertia` (166.7); set `Inertia` too when spin
  matters.
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
phx.Body("Shape", {"path/to/model.stl", "Scale", 0.06*[1 1 1], "Material", "glossy"});
```

Available: `Box`, `Sphere`, `Cylinder`, `Cone`, `Capsule`, `Globe`, `Mesh`,
`Extrusion`, `Revolution`, `Rock`, `Terrain`. Sizing properties are
shape-specific — `Box` uses `Size`; round shapes accept **`Radius`** *or* `Diameter`
(interchangeable), and the axial extent is **`Height`** (not `Length`) with a modeling
`Axis` (`"x"`/`"y"`/`"z"`) on `Cylinder`/`Cone`/`Capsule`. Most accept `Color` and
`Density` (auto-computes mass/inertia if you don't set them), and texture/material
options. `SkeletPoints` adds attachment points used by joints/springs:
`{"Box", "SkeletPoints", [1.5 0 0; -1.5 0 0]}`.

**Appearance.** `Material` picks the shading preset — `"unlit"`, `"matte"`,
`"plastic"` (default), `"glossy"` or `"metal"`. Reach for `"unlit"` when the color
carries data (group, state, a mapped value) rather than shape: it renders the authored
`Color` exactly and ignores the lights, so nothing shades the encoding away. `Style` is
independent of it: `"smooth"` (default), `"edged"`, `"flat"` or `"wireframe"`.

**Imported geometry — `phx.shape.Mesh`.** One class covers all file formats: **STL,
Wavefront OBJ and Stanford PLY** (there is no separate `OBJ`/`STL` shape class). The
cell shorthand recognizes the extension, so `{"model.obj", ...}` /`{"part.stl", ...}`
/`{"scan.ply", ...}` builds a `Mesh`; the explicit form is
`{"Mesh", "Source", "model.obj", ...}`. An OBJ loads as **one merged body** with all
its objects and materials (textures and diffuse colors come from the referenced
`.mtl` automatically); use `Group` to keep a single named object, or
`phx.assembly.import` to turn a multi-object OBJ into separate bodies. Other options:
`Scale`, `Centered` (origin at the bounding-box centre, default true), `Density`,
and for STL `Details` (decimation).

The **collision** geometry is chosen by `Envelope`, and this is the property that
decides whether an imported body behaves:
- `"convex"` (default) — convex hull; the right choice for dynamic bodies.
- `"box"` / `"cylinder"` / `"sphere"` — a fitted bounding primitive; prefer these for
  bodies that must **roll smoothly** (a hull is faceted). `"cylinder"` is aligned
  along `Axis` (`"x"`/`"y"`/`"z"`).
- `"concave"` — the exact triangle mesh; **keep it static** (terrain, funnels, tracks).
  Set `FlipFaces` to reverse the winding if the solid side comes out inverted.

## Prefab assemblies (`phx.assembly.*`)

Package functions that build common multi-body setups in one call. Shared
conventions across all four: an optional **leading axes** target like `phx.Body`
(pass the viewer's `ax`; omitted → `gca`; explicit `[]` → no graphics); the base-pose options
**`Position`** / **`Orientation`** (3×3) / **`EulerAngles`** (z→y→x, alternative
to `Orientation` — combining both errors) that rigidly place the whole assembly
(default: world origin); and plain `phx.Body`/joint objects as return values, so
parts can be restyled or retuned afterwards. Error IDs are per function
(`phx:arena:*`, `phx:chain:*`, `phx:scatter:*`, `phx:import:*`).

```matlab
% Arena: floor + 4 walls. Size = INNER dims [x y z]; origin = middle of the
% floor TOP surface (floor plate below z = 0, walls stand on it).
parts = phx.assembly.arena(ax, "Size", [8 8 1], "Thickness", 0.2);
set(parts.walls, "Color", [0.4 0.55 0.7]);   % parts.floor + parts.walls (-x,+x,-y,+y)

% Chain of rigid links along an N×3 polyline; joints sit at the points.
% Shape "capsule" (default, shortened by one diameter so tips meet) |
% "cylinder" | "box". Axis: 1×3 for all joints or N×3 per point; a zero row
% gives a SphericalJoint (default), non-zero a RevoluteJoint about that axis.
% Anchor "none"|"start"|"end"|"both" pins ends to static mount balls.
p = phx.assembly.chain([0 0 0; 0.4 0 0; 0.8 0 0], "Anchor", "start", "Axis", [0 1 0]);
% p.links (Body array), p.joints (CELL array — joint classes may mix), p.anchors

% Scatter n bodies uniformly in Region [x y z] (spans ±x/2, ±y/2, 0..z — same
% convention as the arena inner space). Spacing > 0 = overlap-free rejection
% sampling (error phx:scatter:regionFull when it does not fit). Draws from the
% GLOBAL rng like rand — reproduce with rng(seed), no state is saved/restored.
% A cell shape spec builds a new shape per body (phx.shape.Rock => variety);
% a shape object is shared by all bodies.
rocks = phx.assembly.scatter({"Rock", "Radius", 0.4}, 30, ...
    "Region", [7 7 4], "Spacing", 0.8, "Position", [0 0 1], ...
    "RandomOrientation", true, "Color", hsv(30));

% Multi-body import, format picked from the extension:
%  .urdf/.xml - one Body per link + joints (robots, ragdolls, vehicles, furniture)
%  .obj       - one Body per Wavefront object (o/g group), no joints; flat
%               objects (ground planes, decals) and line/point objects are
%               SKIPPED (no collision volume). Envelope option applies to all.
%  .stl/.ply  - a single body from the whole mesh
% Bodies come back in a struct keyed by link/object name (makeValidName; the
% original is kept in each object's Name).
[bodies, joints] = phx.assembly.import("human.urdf", ...
    "Position", [0 0 1], "EulerAngles", [0 0.6 0]);
bodies.base.Type = "static";                 % anchor the base when needed

parts = phx.assembly.import("scene.obj", "Envelope", "convex");
```

URDF mapping: revolute/continuous → `phx.RevoluteJoint`, **prismatic →
`phx.PrismaticJoint`**, fixed → `phx.FixedJoint` (planar/floating are substituted
by `FixedJoint` + a warning). Joints are passive (no motors) — actuate them
yourself. Geometries map to `Box`/`Cylinder`/`Sphere`/`Capsule`/`Mesh`; `MeshPath`
resolves `package://` URIs (the URDF's own folder is always searched too).

## phx.Simulation — lifecycle

```matlab
sim = phx.Simulation(ax);             % a specific axes — the viewer's, the usual form
sim = phx.Simulation;                 % all bodies in gca (+ their children)
sim = phx.Simulation(bodies);         % a Body array / cell array (children auto-included)
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

`step` already renders when `redrawStep >= 0`, so **don't add `drawnow` or `pause(0)` to
the stepping loop** — the animation shows up without them and an extra flush per substep
only costs time. Add a single `drawnow` in the *outer* loop iteration only when you have a
reason the scene itself doesn't cover: you draw your own overlay next to the scene (`text`,
`plot`, `viewer.displayText`), or a long run must stay clickable / interruptible while it
proceeds.

## Practical tips (physical tuning)

The skills cover the API; the physics still needs sane numbers. Common pitfalls when
building a scene from scratch:

- **Placing bodies that rest on each other.** There is no "place A on top of B" helper
  — compute the resting pose from shape half-extents (a `Box` of height `h` sitting on
  a surface whose top is at `z_s` has its centre at `z_s + h/2`; a `Sphere` of radius
  `r` rests with its centre at `z_s + r`). If exact contact is fiddly, place the body
  a little above and run a short **settle phase** (a few `step` calls) before the main
  action, letting it drop into contact.
- **Placing many bodies at random.** Don't hand-roll `rand` loops with z-stagger
  tricks against initial overlap — `phx.assembly.scatter` places n bodies in a
  region overlap-free (`Spacing`) in one call.
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
  `Mass` (see the note above) or the lever will feel wrong even with the right masses.
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

- **phx-constraints-forces** — joints, springs, ropes, thrusters, buoyancy, fields,
  `applyForce`/`applyTorque`, scripted and closed-loop control.
- **phx-logging-view** — Logger, Trace, Measure, Zone, Camera, the interactive Viewer, plotting.
- **phx-engine-gotchas** — the `phx.engine.io` gateway, engine variants, error IDs, known quirks, tests.
- **phx-simulink** — drive a PHX scene from Simulink via the PhxModel co-simulation block.
