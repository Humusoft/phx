---
name: phx-engine-gotchas
description: >
  Low-level PHX engine reference and pitfalls — the stringly-typed phxEngine
  gateway (add/set/get/apply/remove/step) to the physics engine,
  phx.engine.Settings, the phx:Component:mnemonic error-ID convention, the
  matlab.unittest suite, known quirks (timestep stability, solver iterations), and
  the accuracy diagnostics of phx.Simulation (DiagnosticInfo, getDiagnosticReport).
  Use when debugging PHX behavior or a scene that misbehaves (stretching joints,
  sinking or tunnelling bodies, jitter, NaN), writing new simulable objects, or
  running the tests.
---

# PHX engine gateway & gotchas

## phxEngine — the single gateway

All physics goes through one stringly-typed MEX dispatcher, `phx/phxEngine`, a
gateway to the physics engine (adapted for engineering use; its sources are not part
of the toolbox).
The first arg is a **verb**, then a world handle and/or object handle, then a field
name and payload. Verbs and field names are raw strings — **typos surface only at
runtime**, so prefer the high-level `phx.*` classes and only drop to `phxEngine` when
implementing a new object.

```matlab
world = phxEngine('step', world, dt);                                    % advance the world
h = phxEngine('add', world, 'sphere', typeID, radius, T, mass, inertia); % create body/shape
phxEngine('set', world, h, 'linvel', v);                                 % mutate a field
v = phxEngine('get', world, h, 'linvel');                                % read a field (1x3 row)
phxEngine('apply', world, h, 'centralforce', F, isLocal);                % apply force/torque
phxEngine('remove', world, h);                                           % destroy
```

Verbs: `add` / `set` / `get` / `apply` / `remove` / `step`. Bodies map to engine
shapes in each `phx.shape.*` class's `createBody` (`'box'`, `'cylinder'`,
`'sphere'`, `'cone'`, `'revolution'`, `'convexhull'`, `'trimesh'`, `'terrain'`, …), points given as 3xN columns and
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
h = phxEngine('add', w, kind, [hA hB], TA, TB, mutualCollisions, extra...);
% kind: 'fixed' 'revolute' 'prismatic' 'spherical' 'generic' 'bushing'
%       'gear' (extra: ratio), 'spring', 'torsionspring' (extra: their settings)
% 'rope' keeps its own form: phxEngine('add', w, 'rope', hs, points, freeLength, k, c)
```

A revolute joint turns about Z of its frames with the zero angle where their X axes
coincide; a prismatic joint slides along Z. Body-level knobs that write straight
through to the engine: `'restitution'` (Body.Restitution, contact bounciness combined
from both bodies) alongside the existing `'friction'`/`'collisions'`.

**Joint fields** (all settable at any time):

```matlab
phxEngine('set', w, h, 'motor', on, targetVel, maxEffort);  % revolute (N*m), prismatic (N)
phxEngine('set', w, h, 'appliedtorque', T);                 % revolute, kept until 0
phxEngine('set', w, h, 'appliedforce', F);                  % prismatic, kept until 0
phxEngine('set', w, h, 'linlimits', lower, upper);          % generic, bushing
phxEngine('set', w, h, 'anglimits', lower, upper);          % generic, bushing (right-handed)
phxEngine('set', w, h, 'springs', kLin, kAng, cLin, cAng);  % bushing
F = phxEngine('get', w, h, 'feedback');                     % 3x4 [fA tA fB tB]
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
phxEngine('apply', w, hs, 'centralforce', F, isLocalForce)       % at the COM
phxEngine('apply', w, hs, 'force',        F, P, isLocalForce, isLocalPoint)
phxEngine('apply', w, hs, 'torque',       T, isLocalTorque)
% hs a 1xN row of body handles (one body: N = 1), F, P, T 3xN (one column
% per body), the flags one for all or one per body
```

Prefer passing the **local** force/torque and a **local** point and letting the engine
rotate them (`Thruster` does this) over pre-rotating by `body.Matrix(1:3,1:3)` in
MATLAB. A local point is taken relative to the body origin; the engine computes the
moment arm about the centre of mass itself. Forces given by `apply` last **one engine step**
(one substep of `sim.step`) and are cleared afterwards. The public wrappers are `Body.applyForce`/`Body.applyTorque`;
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
- **Velocity iterations end at a tolerance.** Each island (bodies that touch or are
  joined) iterates until its joints meet `JointTolerance` (1e-3) and its contacts
  `ContactTolerance` (1e-3), at most `JointIterations` (50, an island with a joint) or
  `ContactIterations` (20, any other; there is no `Iterations` property any more). A
  tolerance is the relative accuracy of the impulses a body receives: the largest change
  one joint/contact still makes to a body's velocity in an iteration, as a fraction of
  what joints and contacts changed it in the whole step (at least |g| dt). Tolerance 0 =
  run to the cap every step (the old fixed count).
- **The joints of an island are solved directly.** Fixed, revolute, prismatic and
  spherical joints, generic joints with all rotations or all translations locked, and
  revolute/prismatic motors below their `MaxTorque`/`MaxForce` are solved together in
  every iteration, closed loops included: a mechanism of them converges in a few
  iterations at any mass ratio and its joint forces are true reactions. Iterative remain
  contacts, motors at their limit, cylindrical/gear/bushing joints, springs and ropes,
  and islands of more than about a hundred joints and contacts (split; `SplitIslands`).
  So the islands at the cap are mechanisms in contact (walkers, vehicles, ragdolls),
  heaps and tall stacks; a lower `JointIterations` speeds them up at a cost in accuracy
  (a coaster's wheel joints opened twice as far at 20 as at 50), a higher one rarely
  helps.
- `PositionIterations` (4) — pull drifted joints and penetrations back after the velocity
  iterations; each takes a fifth of the error, so a joint's drift halves per doubling.
- `Threads` (1) — more pay off from hundreds of bodies; `ContactCache` (off) — faster
  piles at rest, but slowly rolling bodies gain speed, so leave it off.
- Bodies **never sleep** (no setting); `phx.Particles` has its own `Sleeping`.

## Diagnosing a scene — DiagnosticInfo

When a scene misbehaves (a mechanism sags or rings, joints stretch, bodies sink, jitter
or pass through walls, a result changes with the step) **measure before tuning**: turn on
the engine's accuracy diagnostics instead of guessing at `dt` or iterations.

```matlab
sim = phx.Simulation(ax, "DiagnosticInfo", "warnings");  % or set it later, any time
sim.step(0.02, 4, -1);
r = sim.getDiagnosticReport;      % struct of the last call of step; [] while "off"
```

Levels: `"off"` (default, zero cost), `"warnings"` (warnings only, each kind once per
simulation), `"summary"` (+ one line per `step` call), `"detailed"` (+ a few lines per call
naming the bodies and joints). Each report covers the engine steps of one `step` call and
keeps the worst value. Fields:

- `JointResidual`, `ContactResidual` — the largest velocity change one joint / contact still
  made to a body in the solver's last iteration, relative to what joints and contacts
  changed that body's velocity in the step (at least |g| dt): the quantity of
  `JointTolerance` / `ContactTolerance`. At or below the tolerance = converged; above it =
  islands stopped at their cap (`JointIslandsAtCap` / `ContactIslandsAtCap` > 0); `NaN` =
  no joints / contacts. Heaps of contacts rarely converge and sit at their cap as a rule:
  judge them by `Penetration`, not by `ContactResidual` (there is no contact warning).
- `Penetration` (m) + `PenetratingBodies`, `JointGap` (m) + `GapJoint` (fixed, revolute,
  spherical joints), `MaxSpeed` (m/s) + `FastestBody`, `MaxAngularSpeed` +
  `FastestTurningBody`, `NonfiniteSteps` + `NonfiniteBody` — the objects are the scene's
  `phx.*` objects in cell arrays, so you can read their `Name`, pose or mass directly.
- `Iterations`/`MaxIterations` per island, `JointIslandsAtCap` (islands with a joint,
  spring or rope; cap `JointIterations`) and `ContactIslandsAtCap` (contacts only; cap
  `ContactIterations`), summed over the steps, `Islands`/`SplitIslands`,
  `StepTime`/`MaxStepTime` (s, engine only), `ActiveBodies`, `Steps`, `Time`.
- `Warnings` — identifiers of the limits exceeded in this call, even those whose
  console warning was already issued once.

Warnings (`phx:Simulation:*`, lengths relative to `EngineSettings.LengthScale`) and what
to do:

| Identifier | Limit | Remedy |
|---|---|---|
| `nonfinite` | NaN/Inf pose or velocity | the run failed; reduce `dt` |
| `jointsNotConverged` | joint residual > 1 | reduce `dt`, or raise `JointIterations` |
| `initialOverlap` | bodies > 10 % of LengthScale inside each other in the first step after the simulation was made or `addObjects` | place them apart (the engine pushes them out, often violently) |
| `deepPenetration` | later: > 10 % of LengthScale (silent while an initial overlap is pushed out) | reduce `dt`; check `LengthScale`, mass ratios |
| `jointGap` | > 1 % of LengthScale | reduce `dt`, or raise `JointIterations` |
| `fastBody` | > 25 % of LengthScale per engine step | reduce `dt` (tunnelling risk) |

How to use it when debugging:

- Run the suspect part **headless with `"warnings"`**, loop over `step` and collect the
  worst values from `getDiagnosticReport` (e.g. `max` of `JointGap` and the residuals).
  Prefer the report over parsing console text. `"detailed"` is for a human, or for
  finding *which* body or joint is the culprit in a few calls.
- Fix in the order of the remedies: `dt` first, then `LengthScale` for small or large
  parts, the solver last. Residual above the tolerance with islands at the cap → raise
  that cap (`JointIslandsAtCap` → `JointIterations`, `ContactIslandsAtCap` →
  `ContactIterations`); residual at the tolerance but the
  result still too coarse → lower the tolerance. Re-run and compare the numbers; halving
  `dt` until they stop changing is the ground truth.
- A warning is a hint, not a verdict: a body in free fall (a parcel off a conveyor) trips
  `fastBody` harmlessly; silence one with `warning("off", "phx:Simulation:fastBody")`.
  Changing `DiagnosticInfo` re-arms all warnings.
- **Turn it off again** in the final script and in any timing measurement: it costs a few
  per cent of the engine step (results stay bit-identical). Don't leave it on in demos.

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
  stiffness, so a stiff spring alone is no reason for a smaller `dt`. Accuracy, not
  stability, sets two finer limits: `ω·dt ≤ 0.1` for a fast-turning part whose speed or
  bearing forces are the result, and about 1 ms for impacts or rocking whose outcome is
  the result — only for those, not for a whole scene where they are a bulk or secondary
  motion (**phx-scene-basics**, "Finer steps only for what the scene is about").
- **Remedy for a scene that stays soft at a sane `dt`: the solver settings.** Leave
  `phx.engine.Settings` alone in ordinary scenes. Reach for it only when, already at
  `dt ≤ 5 ms`, a stack sinks and drifts (raise `ContactIterations`, default 20) or a
  jointed mechanism in contact, or with joints solved by iterations (above), sags or
  rings (raise `JointIterations`, default 50, up to 100, or
  lower `JointTolerance` if its islands already converge; it costs only the jointed
  groups). None of it helps when the step itself is too long, so lower `dt` first. There
  is no choice of solver. `DiagnosticInfo` (above) tells which of them the scene needs.
- **Static vs kinematic vs dynamic**: `static` bodies have infinite mass and never
  move; `kinematic` bodies are driven by you and push dynamics but ignore forces;
  only `dynamic` bodies integrate forces. A kinematic pose is reached within the next
  engine step and the body then stands still: a pose set once before
  `sim.step(interval, n)` with `n > 1` jerks the body (moves in the first substep, still
  in the others), which shows up as early sliding or toppling of what rides on it. Set
  the pose before every `sim.step(dt)`, or drive by `LinearVelocity`/`AngularVelocity`. Set mass/inertia explicitly for controlled
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
Route **all** engine access through `phxEngine`. Start from
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
