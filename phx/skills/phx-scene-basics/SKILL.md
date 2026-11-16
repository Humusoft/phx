---
name: phx-scene-basics
description: >
  Build and run a physics scene with the PHX MATLAB toolbox (phx.* objects over
  a physics engine). Use when creating phx.Body objects, attaching phx.shape.*
  geometry, soft and flexible bodies or particle groups (phx.SoftBody,
  phx.FlexibleBody, phx.Particles), importing meshes (STL/OBJ/PLY) or URDF models, using the phx.assembly
  prefab builders (arena, chain, scatter, wall, import), drawing the scene into
  phx.extra.Viewer (the default) or plain axes, stepping a phx.Simulation,
  saving a scene to a MAT file or loading one back, or running a PHX scene
  headlessly. Start here before the other phx-* skills.
---

# PHX scene basics

PHX is a MATLAB toolbox (package root `phx/+phx`, requires MATLAB R2025a or newer) — a thin
object-oriented layer over a physics engine reached through one MEX
gateway (`phxEngine`). You build a scene from `phx.*` handle objects that
draw into MATLAB axes via `hgtransform`, then step it with a `phx.Simulation`.

## Path and how to run

PHX is used as an **installed toolbox** — `phx.*` is simply available, no `addpath`
is needed and scene code must never contain one. The `phxex_*` examples are on the path
too, each one a single command.

```shell
# Headless: -batch exits MATLAB when done and returns non-zero on error
matlab -batch "phxex_minimal"
```

Wrap long runs with a timeout — engine demos can take minutes. Graphics-dependent
scenes still run under `-batch` (invisible figures), so headless verification works.

## The scene shape

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
sim.step(1, 100);                            % advance 1 s in 100 substeps
delete(sim);                                 % tear down engine counterparts
```

A typical scene is 5–10 lines. The viewer wants the **whole figure** — it is not made
to share a window — so use plain MATLAB axes instead when:

- the scene is a throwaway/minimal illustration (`phxex_minimal`),
- the visualization must live in a **subplot** or an app layout (`phxex_multisim`),
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
sim.step(1, 100);
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
    "Friction", 0.5, ...                     % Coulomb coefficient >= 0 (no upper bound)
    "Shape", {"Box", "Size", [4 3 0.1], "Color", [1 1 1]});
```

- **Type**: `static` (infinite mass, never moves — floors, walls), `kinematic`
  (you drive its pose or velocity, it pushes others), `dynamic` (engine-driven).
- **Driving a kinematic body**: a pose (`Position`, `EulerAngles`, …) is reached within
  the **next engine step**, then the body stands still. So set the pose before **every**
  step and step without substeps (`sim.step(dt)` in the loop), never once before
  `sim.step(interval, n)`: the body would jump in the first substep and stand still in the
  others, and friction would jerk whatever rides on it. For motion at a steady rate,
  set `LinearVelocity` / `AngularVelocity` once instead: they hold until changed, over
  any number of substeps (set them to zero to stop). A `phx.Script` or `phx.Function`
  writing the pose runs every substep, so it is fine.
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
  bodies** as their product: a bouncy ball dropped on a default floor does **not** bounce —
  give the floor nonzero `Restitution` too. The rebound height is `e²·h` with `e` the
  product of both values (exact at a 1 ms step, within 2 % at 5 ms), so compute `e` from
  the bounce you want.
- **`SurfaceVelocity` — a virtual moving surface (conveyor belts).** A `[vx vy vz]`
  vector, default `[0 0 0]`, settable live. The body itself does **not** move; contacts
  with it are solved as if its surface were sliding at that velocity, so resting bodies
  are dragged along. A belt is therefore an ordinary **static** box with one extra
  number, no scripting and no moving geometry:

  ```matlab
  belt = phx.Body(ax, "Type", "static", "Position", [0 0 -0.1], ...
      "Friction", 0.9, "SurfaceVelocity", [1.2 0 0], ...
      "Shape", {"Box", "Size", [8 2 0.2]});
  ```

  - The vector is in the **body frame**, so a turned or tilted belt drives along its own
    x axis. Build belts along local `+x` and rotate the body; the built-in `"arrows"`
    texture points along local `+x` on the top face, so it then matches the drive for
    free. For any other drive direction turn the arrows with `TextureTransform`:
    `"rot90"` points them along `+y`, `"rot180"` along `-x` and `"rot270"` along `-y`.
  - It is an ordinary contact constraint, so it is **friction-limited**: both bodies need
    a nonzero `Friction`, a frictionless belt transmits nothing however fast it runs.
    Cargo converges to exactly the commanded speed, and the reaction pushes back on the
    belt (a belt free to move recoils, momentum is conserved).
  - Put it on the **moving** body instead and it drives that body the *opposite* way: it
    describes the relative motion of the touching surfaces. That gives a self-propelled
    body, e.g. a tracked vehicle with no track to model. Differential speeds on two such
    tracks do **not** steer it, though.
  - It cannot express a **rotating** surface (it is one constant vector), so turntables
    and rollers spinning about their own axis are out of scope.
  - Building a belt network (see `phxex_beltmaze`): where one belt hands over to another
    running across it, sweep the drive vector ~15 deg sideways so the cargo lands on the
    middle of the next belt instead of riding its edge, and never let a guard rail reach
    into that hand-over square or the cargo jams against the rail's end.
- **Pose** is read/written through these *dependent* properties (they hit the engine
  live): `Position`, `Orientation` (3x3), `Quaternion`, `AxisAngle`, `EulerAngles`,
  `Transform` (4x4). Reading `b.Position` mid-simulation returns the current state.
- **Pointing a body axis**: `XAxis`/`YAxis`/`ZAxis` read a body axis as a unit vector;
  setting one (any nonzero length) turns the body *from its current orientation* by the
  smallest rotation that points that axis there, without twisting about it. The easy
  way to aim a rod, cylinder or thruster: `"ZAxis", p2 - p1`. Name-value pairs apply in
  the order given, so in `phx.Body(ax, "EulerAngles", e, "ZAxis", d)` the axis turns
  the orientation `e` set; a direction opposite to the axis flips the body 180° about
  its next axis (Y for X, Z for Y, X for Z).
- **State you can read**: `LinearVelocity`, `AngularVelocity`, `TotalForce`,
  `TotalTorque` (resultants of all forces in the last engine step, world axes, torque
  about the centre of mass), `Energy`, and `LinearAcceleration`/`AngularAcceleration` (mean over
  the last step, world axes; gravity included, so a falling body reads g).
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
- **`CenterOfMass`** (body frame, from the origin) comes from the shape too: zero for
  box/sphere/cylinder/capsule, on the axis for a cone, the centroid for meshes. The body
  turns about it and gravity acts there, while `Position` stays the origin where the
  shape is drawn. Set it for ballast or cargo (`b.CenterOfMass = [0 0 -0.3]` lowers a
  hull's centre), also mid-run; it changes neither the pose nor `Inertia` (about the
  centre). Like `Mass`, a value given *before* `"Shape"` in the constructor is replaced
  by the shape's — give it after.
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
phx.Body("Shape", {"Cone",     "Radius", 0.3, "Height", 1});  % [0.3 0] = sharp; [0.3 0.1] = truncated
phx.Body("Shape", {"Capsule",  "Radius", 0.25, "Height", 1});
phx.Body("Shape", {"Tube",     "InnerDiameter", [0.1 0.5], "Height", 1});  % funnel; 0.5 = straight
phx.Body("Shape", {"path/to/model.stl", "Scale", 0.06*[1 1 1], "Material", "glossy"});
```

Available: `Box`, `Sphere`, `Cylinder`, `Tube`, `Cone`, `Capsule`, `Mesh`,
`Extrusion`, `Revolution`, `Rock`, `Terrain`. Sizing properties are
shape-specific — `Box` uses `Size`; round shapes accept **`Radius`** *or* `Diameter`
(interchangeable), and the axial extent is **`Height`** (not `Length`) with a modeling
`Axis` (`"x"`/`"y"`/`"z"`) on `Cylinder`/`Cone`/`Capsule`/`Tube`. `Cone` sizes and the `Tube`
bore take a pair, at the negative and the positive end of `Axis`, and always read back as
one: a single cone value `d` is `[d 0]`, a single bore `d` is `[d d]` (the wall keeps its
thickness). Round shapes set their visual
detail with `Segments` (`Sphere` default 28; about 10 is plenty for hundreds of small
particles) — the collision shape stays an ideal primitive either way. Most accept `Color` and
`Density` (auto-computes mass/inertia if you don't set them), and the appearance
options. Shapes carry no attachment points: where a joint or spring grabs a body is set
on the connector itself (`PointA`/`PointB` in body-local coordinates).

**Appearance** — `Color`, `Material` (shading preset), `Style` (edges, wireframe) and
`Texture` (built-in patterns, image files or matrices), with when to use each: see
**phx-logging-view**, "Appearance of shapes". Textures show only in the viewer's axes.

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
- `"concave"` — the exact triangle mesh, the only envelope that keeps cavities and
  holes open (terrain, funnels, tracks). It works for dynamic bodies too, at a higher
  collision cost than a hull.
  Set `FlipFaces` to reverse the winding if the solid side comes out inverted.

`phx.shape.Tube` defaults to `Envelope = "concave"`, unlike every other shape — a
convex hull would seal the bore shut and nothing could be poured through it.

`phx.shape.Revolution` takes `Envelope` too: `"convex"` (default), `"concave"`,
`"cylinder"` fitted about the revolution axis (its radius the largest radius in the
`Profile`), or `"cones"`: the profile revolved exactly, one truncated cone per segment.
Both round envelopes give a **smooth rolling rim** instead of a faceted hull; prefer
`"cones"` for bottles, skittles, pulleys, wheels with a tread or flange, since it keeps
the shape (grooves, waists, tapers). It needs one radius per height: a profile whose
axial coordinates turn back (a cup, a bore, `phx.shape.Tube`) raises
`phx:Revolution:profileNotMonotonic`; use `"concave"` for those. Tens of profile points
are cheap, hundreds cost like a concave mesh. Neither round envelope needs graphics.

What the `Profile` encloses:
- **Ends on the axis** (radius 0 at both ends): a solid body.
- **Closed loop** (last row repeats the first): a **hollow** body — the loop is the wall's
  cross-section. Run it up the outside and back down the inside to give the wall a
  thickness, and use `Envelope = "concave"` (every other envelope fills the cavity). A
  hollow vase with a bottom:
  ```matlab
  t = linspace(0, 1, 20)'; r = 0.3 + 0.2*sin(t*pi);
  profile = [0 0; t r; flipud([t(2:end) r(2:end) - 0.03]); t(2) 0; 0 0];
  phx.Body("Shape", phx.shape.Revolution("Profile", profile, "Envelope", "concave"));
  ```
- **An end off the axis, not a loop**: drawn as an open surface of no thickness, but
  weighed as the solid closed there by a disc square to the axis — what `"convex"`,
  `"cones"` and `"cylinder"` collide with. Not a way to make a hollow part: with
  `"concave"` it collides as a paper-thin shell yet weighs as the solid. A plain straight
  pipe is `phx.shape.Tube`.

## Deformable bodies and particles

Besides rigid `phx.Body` objects a scene can hold three engine objects of their own kind.
Each takes the axes (or `[]`) as its first argument like a body, is **top-level** (hand it
to `phx.Simulation` / `addObjects` like a body, e.g. `phx.Simulation({floor, ball})` or
the axes) and has `boundingBox`. They are newer than the rest of PHX and their tuning is
still in progress: check the result visually, and prefer rigid bodies unless deformation
or a bulk of grains is the point.

```matlab
% Soft body: a triangle mesh whose vertices are particles held by its edges; the mesh
% from a shape (evenly spaced vertices; the shape's geometry only, Mass stays its own)
ball = phx.SoftBody(ax, "Shape", {"Sphere", "Diameter", 0.6, "ElementResolution", 16}, ...
    "Position", [0 0 1], "Mass", 1, "Compliance", 1e-4, "Pressure", 200);   % closed, inflated
% or your own closed mesh, faces counter-clockwise from outside: "Vertices", V, "Faces", convhull(V)

% Flexible body: an elastic solid of quadratic tetrahedra, here a shape filled with
% tetrahedra as fine as its ElementResolution (takes the shape's Density)
block = phx.FlexibleBody(ax, "Shape", {"Box", "Size", [0.4 0.4 0.4]}, ...
    "Position", [1 0 0.5], "YoungsModulus", 2e5, "PoissonsRatio", 0.45);
% or a mesh of any shape from PDE Toolbox: "Mesh", generateMesh(fegeometry("part.stl"), Hmax=0.02)
% or your own: "Vertices", W, "Tetrahedra", T (Kx4, edge vertices added, or Kx10)

% A rigid body overlapping it holds it: vertices inside the body are welded to it
plate = phx.Body(ax, "Type", "static", "Position", [1 0 0.7], ...
    "Shape", {"Box", "Size", [0.6 0.6 0.05]});
phx.IntersectionWeld(block, plate);

% Particles: thousands of equal spheres simulated and drawn in bulk
gravel = phx.Particles(ax, "Positions", P, "Radius", 0.02, "Density", 1600, ...
    "Color", [0.8 0.6 0.4], "RandomTint", 0.4, "MarkerSize", 8);
gravel.add(Pnew); gravel.remove(gravel.Positions(:, 3) < -1);   % any time, also mid-run
```

- `SoftBody` (cloth, cushion, inflated ball) and `FlexibleBody` (rubber block, pad,
  bushing) collide with rigid bodies only, not with each other. **No joint attaches to
  them**: weld a `phx.Body` into them with `phx.IntersectionWeld` (it should overlap at
  least 3 vertices not on one line) and connect joints, springs and forces to that body.
- `"Shape"` meshes a `phx.shape.*` object or a `{name, args...}` cell (not together with
  a mesh). `Box`, `Sphere`, `Cylinder`, `Cone`, `Capsule` and `Tube` generate meshes;
  `Revolution`, `Extrusion` and `Mesh` raise `phx:Shape:noVolumetricMesh` /
  `noSurfaceMesh` until a future release (`Rock`, `Terrain` too): give those a PDE Toolbox
  `"Mesh"` or `Vertices` of your own. A FlexibleBody takes the shape's `Density` (a
  `Density`/`Mass` given to the constructor wins), a SoftBody only its geometry.
- `ElementResolution` of the shape (like `Segments` for drawing) sets the fineness as cell
  counts, independent of the size. A scalar counts cells across the thinnest section, the
  rest near-cubic: Box smallest side, Sphere the diameter, Cylinder/Cone the smaller of
  (larger) diameter and height, Capsule the diameter, Tube the thinner of wall and height.
  A vector gives the counts per direction: Box `[nx ny nz]` (read back by
  `ElementDivisions`), Cylinder/Cone `[diameter height]`, Capsule `[diameter Height]`,
  Tube `[across around along]`; Sphere takes a scalar only. Defaults suit a FlexibleBody
  (cheap; a beam bends within ~5 % of beam theory): Box/Cylinder/Cone/Capsule 2, Sphere 5,
  Tube `[1 24 8]`. A SoftBody usually wants finer (e.g. 16 for a ball); a scalar on a
  thin plate or wall makes very many cubes, so give a vector there.
- Round shapes give Kx10 tetrahedra whose surface edge middles lie on the true surface,
  so even a coarse mesh has nearly the shape's volume and mass. Soft-body surfaces space
  their vertices evenly (geodesic sphere, rings for the other round shapes).
  `generateVolumetricMesh` / `generateSurfaceMesh` of the shape return `[V, T]` /
  `[V, F]` themselves, e.g. to scale a sphere surface into an ellipsoid.
- The mesh is the rest shape, placed at `Position`/`Orientation` (or `EulerAngles`,
  `AxisAngle`, `Quaternion`, as on `Body`) when the simulation starts; mesh and material
  are read then, the friction/damping/pressure settings at any time. `CurrentVertices`
  gives the deformed mesh; `FlexibleBody.VonMisesStress` the stress per vertex (colour it
  with `Colormap`, an N×3 matrix such as `turbo(64)`, and `ColorRange`).
- `FlexibleBody` uses quadratic tetrahedra (a Kx4 mesh gets edge vertices; `ElementOrder
  "linear"` takes the corners, also of a Kx10 mesh): implicit,
  stiffness right on a coarse mesh (2-3 elements across a section to bend, 3-4 to twist),
  `Tolerance` sets the convergence and `Iterations` caps it. Its contacts are inelastic
  (no `Restitution`). Good for large rotations with strains up to some 10-20 %.
  It touches rigid bodies at its vertices only: keep them well closer together than the
  radius of a curved body it touches.
- Masses: `FlexibleBody.Mass` (total) and `Particles.Mass` (total) / `ParticleMass` (one
  particle) are settable views of `Density`; set one of them, not both.
- `Particles`: at most 10 000 per group (use more groups); `Positions`/`Velocities` are
  N×3 arrays, `applyForce(F)` (world frame; 1×3 = the whole F on every particle, N×3 = one row
  each) acts in the next engine step (one substep) of a running simulation. Set the
  engine's `LengthScale` to the grain size class (`phx.engine.Settings("LengthScale",
  0.01)` for centimetre grains).

## Prefab assemblies (`phx.assembly.*`)

Package functions that build common multi-body setups in one call. Shared
conventions across all five: an optional **leading axes** target like `phx.Body`
(pass the viewer's `ax`; omitted → `gca`; explicit `[]` → no graphics); the base-pose options
**`Position`** / **`Orientation`** (3×3) / **`EulerAngles`** (z→y→x, alternative
to `Orientation` — combining both errors) that rigidly place the whole assembly
(default: world origin); and plain `phx.Body`/joint objects as return values, so
parts can be restyled or retuned afterwards. Error IDs are per function
(`phx:arena:*`, `phx:chain:*`, `phx:scatter:*`, `phx:wall:*`, `phx:import:*`).

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
% RandomTint (0..1, default 0) darkens each body on its own like in wall:
% bodyColor = color*(1 - rand*RandomTint) over the Color row OR, with no Color,
% over the body's palette color (that palette cycles per created shape, NOT
% from the rng — so seeding reproduces the layout and the tint, not the base hue).
rocks = phx.assembly.scatter({"Rock", "Radius", 0.4}, 30, ...
    "Region", [7 7 4], "Spacing", 0.8, "Position", [0 0 1], ...
    "RandomOrientation", true, "Color", hsv(30));

% Wall of loose bricks (no mortar) in a running bond, origin = middle of the
% BASE, spans ±x/2 in length, ±y/2 in thickness, 0..z up. Size is the WHOLE
% wall; the brick size follows: x/Columns long, y thick, z/Rows high.
% HalfBricks true = even rows closed by half bricks at both sides (flush ends,
% Columns+1 bricks); false = left open (Columns-1 bricks, inset half a brick,
% toothed edge; needs Columns >= 2 => phx:wall:invalidColumns).
% RandomTint (0..1, default 0.1) shades each brick on its own:
% brickColor = Color*(1 - rand*RandomTint) — GLOBAL rng like scatter, 0 = off
% (and no rand call at all). set(bricks,"Color",...) afterwards WIPES the tint.
% Returns a 1xn Body array, bottom row first and left to right, named
% "brick<row>_<col>". Stands on friction alone — put it on a floor first.
bricks = phx.assembly.wall(ax, "Size", [3 0.25 1.2], "Rows", 10, "Columns", 6, ...
    "HalfBricks", true, "Density", 2000, "Color", [0.7 0.35 0.25], "RandomTint", 0.3);

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
`phx.PrismaticJoint`**, fixed → `phx.FixedJoint` (planar → `GenericJoint` + a
warning, floating → no joint, the child left free). The `<limit lower upper>` of a
joint (a rigidBodyTree's `PositionLimits`) becomes its `Limits` end stops; continuous
joints, full-turn ranges and ranges a PHX hinge cannot bound (reaching past ±π) stay
free, the last with a `phx:import:jointLimitsIgnored` warning; `"JointLimits", false`
imports every joint free. `Angle`/`Distance` read the URDF joint position. The imported
joints come with their motors off; drive them by setting `TargetVelocity` + `MaxTorque`
(revolute) or `MaxForce` (prismatic) on the joints the importer returns — see the
phx-constraints-forces skill. Nothing in the URDF sets those, so the motor limits are
yours to size.
Geometries map to `Box`/`Cylinder`/`Sphere`/`Capsule`/`Mesh`; `MeshPath`
resolves `package://` URIs (the URDF's own folder is always searched too).

## phx.Simulation — lifecycle

```matlab
sim = phx.Simulation(ax);             % a specific axes — the viewer's, the usual form
sim = phx.Simulation;                 % all bodies in gca (+ their children)
sim = phx.Simulation(bodies);         % a Body array / cell array (children auto-included)
sim = phx.Simulation("scene.mat");    % bodies saved in a MAT file (see Saving & loading)

sim.Gravity = [0 0 -9.81];            % default; settable
sim.step(interval, substeps);
sim.addObjects(moreBodies);           % rebuilds pipelines
delete(sim);
```

`step(interval, substeps, redrawStep)`:
- `interval` — seconds of simulated time advanced by this call.
- `substeps` — number of physics substeps; the engine timestep is `dt = interval/substeps`.
- `redrawStep` — optional, **usually left out**. Default `1` = a redraw may be requested on
  every substep; `0` = no redraw during the step (the graphics are only brought up to date
  at its end); `-1` = headless, the graphics are not touched at all.

Run it repeatedly to interleave control logic between steps (read body state, set an
actuator, step again) — see the **phx-constraints-forces** skill for the control-loop
pattern. How small `dt` must be: "Practical tips" below.

`sim.RedrawMode` decides how — and in the default mode also how often — the scene is
redrawn:
- `"adaptive"` — the default. Redraws as often as the screen and the cost of a redraw
  allow (at most once per display frame, drawing never takes more than about half the
  run) and skips the rest, so frame-thinning by hand is unnecessary: **don't pass a
  `redrawStep` > 1 and don't throttle with `mod(k, N)`**. The figure stays live during the
  run (orbit/pan/zoom, select and drag bodies, the viewer's keys and app buttons work).
  While the simulation runs ahead of real time, a frame is still drawn at least every
  `sim.FrameSpacing` seconds of *simulation* time (default 1/20 s), so a light scene is
  seen moving. For long time scales (orbits, thousands of simulated seconds) raise it,
  e.g. `"FrameSpacing", 1` for one frame per simulated second, or the run spends its time
  drawing.
- `"full"` — every requested redraw is drawn and waited for (`drawnow`); the slowest. Use it
  when every frame matters — recording a video with `phx.Camera`, frame-by-frame figures.
- `"performance"` — every requested redraw is sent without waiting and without servicing
  callbacks (the window doesn't react while a step runs); the lowest and most predictable
  cost per frame. Use it for benchmarks and timing measurements, with an explicit
  `redrawStep` (e.g. `sim.step(5, 1000, 10)` = a frame every 10th substep).
- `"none"` — no screen update at all; the graphics objects are still updated, so the
  figure catches up once MATLAB goes idle. **Not a headless shortcut** — the graphics are
  still brought up to date on every requested redraw, so a headless run wants
  `redrawStep = -1`, several times cheaper per substep.

```matlab
sim = phx.Simulation(ax, "RedrawMode", "performance");   % a timing measurement
sim.step(5, 1000, 10);                                    % fixed: a frame every 10th substep
```

The simulation itself — and so every result — is the same in all modes; only the drawing
differs.

Because the default services callbacks, an animated `step` is a **re-entrancy point**: a
button callback, key press or timer can fire in the middle of it. Don't `delete` the
simulation or its bodies from such a callback — set a flag and act on it between steps.

`step` already renders when `redrawStep > 0`, so **don't add `drawnow` or `pause(0)` to
the stepping loop** — the animation shows up without them, and keeping the figure
responsive is `RedrawMode`'s job. Your own overlays next to the scene (`text`, `plot`,
`viewer.displayText`) updated between steps reach the screen with the next redraw; a
single `drawnow` is needed only after the last `step` when the code goes on computing.

`sim.PacingRate` caps the speed of the run: `1` = at most real time, `0.5` = half speed,
`Inf` (the default) = as fast as the computer allows. The wait happens at each requested
redraw and at the end of each `step` (a `redrawStep` of 0 or -1 waits only at its end);
delays up to 0.1 s are made up, a scene that falls further behind just runs as fast as it
can and never speeds up to catch up. The wait is a `pause`, so even under
`"performance"` the window reacts while it waits.
**Don't set it on your own.** When a simple scene — a falling ball, a pendulum, an
educational demo — will be over faster than the eye can follow, offer it to the user as an
option in one line of your reply (e.g. "it runs about 10x faster than real time; add
`"PacingRate", 1` to watch it at real speed") and add it only if they want it. Never pace
headless runs, parameter sweeps or anything that reports a timing number.

```matlab
sim = phx.Simulation(ax, "PacingRate", 1);   % at most real time — only on the user's request
```

## Saving and loading scenes (MAT files)

A scene is just `phx.Body` handle objects, so plain `save`/`load` is the whole mechanism —
there is no export format. **Save the bodies**: everything attached to one (shapes, joints,
springs, ropes, force elements, loggers, zones) is a *child* of that body and is written
with it. Variable names are irrelevant on the way back in — loading picks every top-level
object (`phx.Body` or anything else no other object owns) in the file and ignores the rest,
including cell arrays.

```matlab
save("scene.mat", "ground", "arm");            % the joint between them travels along
objs = phx.Simulation.findTopLevel(ax);        % or grab whatever is drawn in an axes (a cell)
bodies = [objs{:}];                            % all bodies here: join them, a cell is not loaded
save("scene.mat", "bodies");                   % (the viewer's Save model writes one variable per class)

model.chassis = chassis; model.wheels = [wFL wFR wRL wRR];
save("saved_buggy.mat", "-struct", "model");   % keeps the parts named
```

**Never save a `phx.Simulation`** — it owns a live engine world; the file reloads as a
broken object (`Unrecognized field name "updatePipelines"` on load, then an
error in the destructor).

```matlab
sim = phx.Simulation("scene.mat");   % loads the file itself; works headless
sim.propagate("ParentAxes", ax);     % loaded bodies are UNPARENTED until you do this
sim.addObjects("other.mat");         % a file can also be dropped into a running sim

model = load("saved_buggy.mat");     % keep the struct when parts must stay addressable
propagate(cell2mat(struct2cell(model)), "ParentAxes", ax);
groupTransform(cell2mat(struct2cell(model)), "Translation", [-3 0 0]);
sim = phx.Simulation(model);         % a struct source is scanned for its phx.Body fields
```

What survives: the object tree, all properties, the graphics (colors/textures included),
and the **full kinematic state** — pose plus linear and angular velocity, whether you save
mid-run or after `delete(sim)` (velocities are read from the engine as the file is
written; `delete(sim)` also copies them back into the objects). They are pushed back into
the engine on the next run, so a reloaded scene *continues* instead of restarting from
rest.
Engine handles are transient and the pipelines are rebuilt — nothing else to restore.

Convention in `examples/`: `model_*.m` builds the bodies and saves them, `phxex_*.m` loads
the MAT file, parents it into the viewer and simulates it.

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
- **`Friction` is one Coulomb coefficient `>= 0` (no upper bound)**, ≈0.5–0.8 for grippy
  contact, 0 for near-frictionless, above 1 for very grippy contact; a contact multiplies the
  values of both bodies. (Code with the `[drag roll spin]` triplet of PHX 1.x: see
  **phx-upgrade-from-1x**.)
- **Rolling and spinning resistance are lengths in m, both 0 by default**, and a contact
  adds the values of both bodies. `RollingResistance` b: a rolling contact meets a torque
  of up to `b*N` (a ball slows by `b*g/(R*(1 + I/(m*R^2)))`); steel on steel 1e-5 m, wood
  ~5e-4 m, a tyre a few mm. `SpinRadius` r: a point contact (ball on a floor) resists
  spinning about its normal by `Friction*r*N`, about 2/3 of the real patch radius; flat
  faces resist spinning by their own extent anyway. Without them a ball rolls and a top
  spins on forever; set them only where the scene needs it (Euler's disk, a ball that
  must come to rest, granular piles), and keep b small against the body size, as it also
  resists tipping.
- **Tuning a launch / lever / impact.** Nothing predicts the outcome for you. For a
  seesaw-style launch, make the dropped mass clearly heavier than the launched body
  and give it a real drop height, then **verify by logging the launched body's apex**
  (`max(body.Position(3))` across the run) and iterate. Keep `Inertia` consistent with
  `Mass` (see the note above) or the lever will feel wrong even with the right masses.
- **Choosing the step — the rule all phx-* skills refer to.** Keep
  `dt = interval/substeps ≤ 5 ms` for contact-heavy or tightly constrained scenes (joint
  chains, stacks, mechanisms in contact); raise `substeps` rather than taking bigger
  steps. Springs, ropes and torsion springs are no reason for a smaller `dt`: the engine
  solves them stably at any stiffness.
- **Finer steps only for what the scene is about.** Two motions need a shorter `dt` than
  the 5 ms above, but only when they are the point of the scene (what it measures, plots
  or shows off):
  - a part whose spin, bearing forces or driven motion is the result: at most about
    0.1 rad per substep, `ω·dt ≤ 0.1`, for ~1 % (a rotor at 3000 rpm wants 0.3 ms);
  - an impact, rebound height, rocking or toppling amplitude that is the result: about
    1 ms (at 5–10 ms each impact takes some 15–30 % too much energy).

  Do **not** lower `dt` of a whole scene because such motions occur in a bulk process or
  in secondary parts (parts tumbling in a hopper, a poured pile, debris, the wheels of a
  vehicle whose path is the result): the default step keeps them plausible and a finer one
  only costs run time.
- **Small parts.** For parts of a few centimetres whose contacts matter (rolling,
  resting, stacking), set `phx.engine.Settings("LengthScale", 0.01)` (0.1 for
  decimetres) as `EngineSettings`; at the default scale of 1 m they slip and creep.
- **A scene that misbehaves** (joints stretch, bodies sink or tunnel, jitter): measure
  first with `phx.Simulation(..., "DiagnosticInfo", "warnings")` and
  `sim.getDiagnosticReport` — warnings with remedies, residuals, penetration, joint gaps
  and the bodies concerned. Details in **phx-engine-gotchas**; switch it off when done.

## How the object model works (only when subclassing)

Every simulable object subclasses `phx.base.Object`, which holds a DAG (`Parents`/
`Children`), an `hgtransform` (`Graphics`) and an engine handle (`ObjectHandle`).
Subclasses implement four methods: `initObject(obj, world)` (build/refresh engine
counterpart; return `false` to exclude), `destroyObject(obj)`, and the **static,
batch-by-class** `resolveState(cellObjs, dt, time, world)` and
`updateView(cellObjs, dt, time, world)`. New objects route **all** engine access
through `phxEngine` — never call the engine any other way. Use the `phx.template.Interaction`
class as a starting point.

## Related skills

- **phx-constraints-forces** — joints, springs, ropes, thrusters, buoyancy, fields,
  `applyForce`/`applyTorque`, scripted and closed-loop control.
- **phx-logging-view** — Logger, Trace, Measure, Zone, Raycast, PlanarShadow, Camera, the interactive Viewer, plotting, and the appearance of shapes (colors, materials, textures).
- **phx-engine-gotchas** — accuracy diagnostics, engine settings, known quirks, error IDs, the `phxEngine` gateway.
- **phx-simulink** — drive a PHX scene from Simulink via the PhxModel co-simulation block.
- **phx-upgrade-from-1x** — converting code, saved scenes and Simulink models written for PHX 1.x.
