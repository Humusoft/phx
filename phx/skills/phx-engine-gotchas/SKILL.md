---
name: phx-engine-gotchas
description: >
  Low-level PHX engine reference and pitfalls — the stringly-typed phx.engine.io
  gateway (add/set/get/apply/remove/step) to the Jolt Physics engine,
  phx.engine.Settings, the phx:Component:mnemonic error-ID convention, the
  matlab.unittest suite, and known quirks (timestep stability, solver iterations).
  Use when debugging PHX behavior,
  writing new simulable objects, or running the tests.
---

# PHX engine gateway & gotchas

## phx.engine.io — the single gateway

All physics goes through one stringly-typed MEX dispatcher in `phx/+phx/+engine/`, a
gateway to the Jolt Physics engine (adapted for engineering use; its sources are not part
of the toolbox).
The first arg is a **verb**, then a world handle and/or object handle, then a field
name and payload. Verbs and field names are raw strings — **typos surface only at
runtime**, so prefer the high-level `phx.*` classes and only drop to `io` when
implementing a new object.

```matlab
world = phx.engine.io('step', world, dt);                                    % advance the world
h = phx.engine.io('add', world, 'sphere', typeID, radius, T, mass, inertia); % create body/shape
phx.engine.io('set', world, h, 'linvel', v);                                 % mutate a field
v = phx.engine.io('get', world, h, 'linvel');                                % read a field (1x3 row)
phx.engine.io('apply', world, h, 'centralforce', F, isLocal);                % apply force/torque
phx.engine.io('remove', world, h);                                           % destroy
```

Verbs: `add` / `set` / `get` / `apply` / `remove` / `step`. Bodies map to engine
shapes in each `phx.shape.*` class's `createBody` (`'box'`, `'cylinder'`,
`'sphere'`, `'convexhull'`, `'trimesh'`, `'terrain'`, …), points given as 3xN columns and
indices (faces, tetrahedra, particles) as one-based doubles; joints map to kinds named after their
classes in the joint classes. Every failure is a MATLAB error with a `phx:engine:*` identifier
(`badWorld`, `badHandle`, `badArgument`, `unsupported` for an unknown verb, kind or
field); verbs that only act (`set`, `apply`, `remove`, `clear`) return nothing, and
vectors come out as 1x3 rows (batches as 3xN).

**Every joint is added the same way**, with the two body handles as one `[hA hB]`
argument, the full joint frames (4x4, in the body frames, joint axis = Z) and a positive
`mutualCollisions` flag (`true` = the two bodies collide; the classes pass
`obj.MutualCollisions`):

```matlab
h = phx.engine.io('add', w, kind, [hA hB], TA, TB, mutualCollisions, extra...);
% kind: 'fixed' 'revolute' 'prismatic' 'spherical' 'generic' 'bushing'
%       'gear' (extra: ratio), 'spring', 'torsionspring' (extra: their settings)
% 'rope' keeps its own form: io('add', w, 'rope', hs, points, freeLength, k, c)
```

A revolute joint turns about Z of its frames with the zero angle where their X axes
coincide; a prismatic joint slides along Z. Body-level knobs that write straight
through to the engine: `'restitution'` (Body.Restitution, contact bounciness combined
from both bodies) alongside the existing `'friction'`/`'collisions'`.

**Joint fields** (all settable at any time):

```matlab
phx.engine.io('set', w, h, 'motor', on, targetVel, maxEffort);  % revolute (N*m), prismatic (N)
phx.engine.io('set', w, h, 'appliedtorque', T);                 % revolute, kept until 0
phx.engine.io('set', w, h, 'appliedforce', F);                  % prismatic, kept until 0
phx.engine.io('set', w, h, 'linlimits', lower, upper);          % generic, bushing
phx.engine.io('set', w, h, 'anglimits', lower, upper);          % generic, bushing (right-handed)
phx.engine.io('set', w, h, 'springs', kLin, kAng, cLin, cAng);  % bushing
F = phx.engine.io('get', w, h, 'feedback');                     % 3x4 [fA tA fB tB]
```

The motor and the applied torque/force act together. `phx.base.Joint` works `Angle` and
`Distance` out in MATLAB from the poses, with no engine call, and reads `ForceA`…`TorqueB`
as columns of one `'feedback'` call (world frame, torques about each body's centre of
mass, applied loads included). A `phx.Body`'s `AppliedForce`/`AppliedTorque` are the body
fields `'appliedforce'`/`'appliedtorque'` (body frame), kept by the engine until set to
zero.

**`apply` fields and their frame flags** — the payload layout differs per field, and
getting it wrong silently applies a force in the wrong frame (this caused real bugs in
`Thruster`/`Dipole`, now locked in by `tForceApplication`):

```matlab
io('apply', w, hs, 'centralforce', F, isLocalForce)              % at the COM
io('apply', w, hs, 'force',        F, P, isLocalForce, isLocalPoint)
io('apply', w, hs, 'torque',       T, isLocalTorque)
% hs a 1xN row of body handles (one body: N = 1), F, P, T 3xN (one column
% per body), the flags one for all or one per body
```

Prefer passing the **local** force/torque and a **local** point and letting the engine
rotate them (`Thruster` does this) over pre-rotating by `body.Matrix(1:3,1:3)` in
MATLAB. A local point is taken relative to the body origin; the engine computes the
moment arm about the centre of mass itself. Forces given by `apply` last **one step** and
are cleared afterwards. The public wrappers are `Body.applyForce`/`Body.applyTorque`;
for a load that should stay, set `Body.AppliedForce`/`AppliedTorque` instead.

## phx.engine.Settings — initial engine config

Pass initial engine settings via the simulation's `EngineSettings`; they are read once,
as the world is created:

```matlab
sim = phx.Simulation(ax, "EngineSettings", ...
    phx.engine.Settings("LengthScale", 0.01, "Threads", 4));
```

- `LengthScale` — the size class of the parts that touch (1 m, 0.1 for decimetres, 0.01
  for centimetres). The collision tolerances follow it in automatic mode: `Margin` (4 %,
  the rounding of convex shapes, taken inside them), `SpeculativeDistance` (2 %) and
  `PenetrationSlop` (0.1 %). Setting one switches it to manual, like MATLAB axis limits.
  Set `LengthScale` for small parts instead of tuning the tolerances one by one.
- `Iterations` (10) — velocity iterations for contacts and stacks; `JointIterations` (50)
  — those of every group of bodies held by joints (motors hold, chains stay stiff);
  `PositionIterations` (2).
- `Threads` (1) — more pay off from hundreds of bodies; `ContactCache` (off) — faster
  piles at rest, but slowly rolling bodies gain speed, so leave it off.
- Bodies **never sleep** (no setting); `phx.Particles` has its own `Sleeping`.

## Error-ID convention

Errors use a unified `phx:Component:mnemonic` namespace (e.g.
`phx:Simulation:unsupportedSource`) and are selectively catchable/suppressible.
Match on these IDs in `try/catch` rather than message text. When raising errors in
new code, follow the same namespace.

## Known quirks (learned the hard way)

- **Object graph changes rebuild the pipelines automatically.** `phx.Simulation`
  groups objects by class into compute/redraw pipelines. Adding (`sim.addObjects(...)`)
  and deleting (`delete(obj)`) an object both call `updatePipelines` for you — so
  add/delete mid-run "just works", no manual rebuild needed. `delete(joint)` removes
  the engine constraint immediately (the freed body falls that instant); bodies keep
  their `ObjectHandle` and engine state across the rebuild. Only caveat: go through
  `addObjects` / `delete` rather than mutating the `Children` cell array by hand.
- **Tight constraint networks need a small timestep.** Joint chains and
  stacked/contact-heavy scenes want `dt = interval/substeps ≤ 5 ms` for stability —
  raise `substeps` (or lower `interval`) rather than taking big steps. Springs, ropes
  and torsion springs are the exception: the engine solves them stably at any
  stiffness, so a stiff spring alone is no reason for a smaller `dt`.
- **Remedy for a scene that stays soft at a sane `dt`: solver iterations.** Leave
  `phx.engine.Settings` alone in ordinary scenes. Reach for it only when, already at
  `dt ≤ 5 ms`, a stack sinks and drifts (raise `Iterations`, default 10) or a jointed
  mechanism sags or rings (raise `JointIterations`, default 50, up to 100; it costs
  only the jointed groups). Neither helps when the step itself is too long, so lower
  `dt` first. There is no choice of solver.
- **Static vs kinematic vs dynamic**: `static` bodies have infinite mass and never
  move; `kinematic` bodies are driven by you and push dynamics but ignore forces;
  only `dynamic` bodies integrate forces. Set mass/inertia explicitly for controlled
  vehicles instead of relying on shape density.
- **Negative time steps are refused** (`phx:engine:badArgument`): the engine only steps
  forward.

## Tests — matlab.unittest suite

Class-based suite in `tests/` — it ships with the **source repository**, not with the
installed toolbox. Run from that directory (`runtests_phx` puts `../phx` on the path
for the run):

```matlab
cd tests
runtests_phx                 % everything
runtests_phx("noengine")     % skip MEX integration tests
runtests_phx("pure")         % headless: no engine, no graphics
```

Only two tags remain: `Engine` (needs the MEX) and `Graphics` (needs a display-capable
session / `hgtransform`). Engine tests are **assumed away** (filtered, not failed) when
the MEX is absent — a green "pure" run does not prove the engine path works. Tags are
per test method, so most files span layers. The table of test files, their layers and
what each covers, and the areas not covered yet, are in `tests/README.md`; keep that
table current when adding a test file.

## Writing a new simulable object

Subclass `phx.base.Object` (or `Body`/`Joint`/`Shape`) and implement the four
methods (`initObject`, `destroyObject`, static `resolveState`, static `updateView`).
Route **all** engine access through `phx.engine.io`. Start from
`phx.template.Interaction`.

For per-step logic that does **not** need a new class, `phx.Function` (a callback in
the pipeline) is the cheaper answer — see phx-constraints-forces.

The undocumented MATLAB internals PHX uses are **deliberate performance choices** —
they skip argument validation in per-substep hot paths — and `tInternals` guards each
one (`matlab.internal.math.ismemberhelper`, `matlab.internal.math.interp1`,
`matlab.internal.meshio.stlread`, `matlab.io.internal.validators.validateFileName`,
`matlab.graphics.internal.drawnow.startUpdate`, and the `Matrix_I` hgtransform
property) so a MATLAB upgrade fails loudly there instead of in a user's simulation.
Don't propose replacing them with documented equivalents; just don't widen the
reliance — new code stays on the documented surface unless there is a measured reason.
(`robotics.internal.*` is gone — rotations are done by `phx.internal.Math`, so PHX
needs no add-on toolbox.)

## Related skills

- **phx-scene-basics** — the high-level workflow that sits on top of this gateway.
- **phx-constraints-forces** — joints/springs/thrusters and the control loop.
- **phx-logging-view** — recording and visualizing results.
- **phx-simulink** — the PhxModel co-simulation block and its property-reference ports.
