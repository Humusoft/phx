---
name: phx-engine-gotchas
description: >
  Low-level PHX engine reference and pitfalls — the stringly-typed phx.engine.io
  gateway (add/set/get/apply/remove/step), phx.engine.BulletSettings, the
  phx:Component:mnemonic error-ID convention, the matlab.unittest suite, and known
  quirks (timestep stability, body sleeping). Use when debugging PHX behavior,
  writing new simulable objects, or running the tests.
---

# PHX engine gateway & gotchas

## phx.engine.io — the single gateway

All physics goes through one stringly-typed MEX dispatcher in `phx/+phx/+engine/`.
The first arg is a **verb**, then a world handle and/or object handle, then a field
name and payload. Verbs and field names are raw strings — **typos surface only at
runtime**, so prefer the high-level `phx.*` classes and only drop to `io` when
implementing a new object.

```matlab
world = phx.engine.io('step', world, dt);                                    % advance the world
h = phx.engine.io('add', world, 'sphere', typeID, radius, T, mass, inertia); % create body/shape
phx.engine.io('set', world, h, 'linvel', v);                                 % mutate a field
v = phx.engine.io('get', world, h, 'linvel')';                               % read a field
phx.engine.io('apply', world, h, 'centralforce', F, isLocal);                % apply force/torque
phx.engine.io('remove', world, h);                                           % destroy
```

Verbs: `add` / `set` / `get` / `apply` / `remove` / `step`. Bodies map to engine
shapes in each `phx.shape.*` class's `initObject` (`'box'`, `'cylinder'`,
`'sphere'`, `'convexhull'`, `'rigidbody'`, …); joints map to `'*constraint'` verbs
in the joint classes. Enum-like inputs (`Type`→`TypeID`, axes, solver) **are**
validated MATLAB-side; field names and payloads are not.

**All six `'*constraint'` add verbs require a trailing `disableCollisions` logical**
— without it the call fails with "ArgumentList index out of range". The joint
classes pass `~obj.MutualCollisions` there. Body-level knobs that
write straight through to the engine: `'restitution'` (Body.Restitution, contact
bounciness combined from both bodies) alongside the existing `'friction'`/`'collisions'`.

## Engine variants (internal)

The active MEX can be swapped for an alternate build (`debug`, `verbose`, `time`,
plus multithreaded `...Mt` variants) via the internal `phx.engine.switchEngine`
helper. This is an **internal/advanced facility, not part of the typical user
workflow** — mentioned only so you recognize it; don't reach for it in normal
scene-building code. The default `release` build is what runs unless someone has
deliberately switched it.

## phx.engine.BulletSettings — initial engine config

Pass engine-specific initial settings via the simulation's `EngineSettings`:

```matlab
sim = phx.Simulation(ax, "EngineSettings", ...
    phx.engine.BulletSettings("AutoActivated", true));   % opt back INTO sleeping
```

`AutoActivated` controls Bullet's auto-deactivation of resting bodies. **It defaults
to `false`** (bodies never sleep) — the predictable, no-surprises behavior you want
out of the box. Set it to `true` only as a deliberate optimization for large,
mostly-resting scenes; the speedup from sleeping is usually negligible, so leaving
the default alone is almost always right. If you do opt into `true`, mind the
wake-map quirk below.

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
- **Tight constraint networks need a small timestep.** Joint chains, ropes, and
  stacked/contact-heavy scenes want `dt = interval/substeps ≤ 5 ms` for stability —
  raise `substeps` (or lower `interval`) rather than taking big steps.
- **Static vs kinematic vs dynamic**: `static` bodies have infinite mass and never
  move; `kinematic` bodies are driven by you and push dynamics but ignore forces;
  only `dynamic` bodies integrate forces. Set mass/inertia explicitly for controlled
  vehicles instead of relying on shape density.
- **Body sleeping / the wake-map** (only relevant if you set `AutoActivated=true`;
  the default `false` sidesteps all of this). A resting body auto-deactivates after
  ~2 s (its `LinearVelocity` snaps to *exactly* 0 — a reliable "is it asleep?" probe).
  A sleeper is re-activated when something touches it **directly**: any applied
  force/torque (so all field/spring/rope/thruster elements are fine), a velocity or
  transform write, or a new contact from a dynamic *or* kinematic body. It is **NOT**
  woken by changes to its *environment*: changing world gravity (`sim.Gravity = …`)
  or moving/removing the static support under it both leave the sleeper frozen (the
  classic Bullet "body floating in mid-air"). There is no `activate`/`wake` method —
  the only nudge is a self-write like `b.Position = b.Position`. The default `false`
  avoids all of this.

## Tests — matlab.unittest suite

Class-based suite in `tests/`. Run from that directory (`runtests_phx` puts `../phx`
on the path for the run):

```matlab
cd tests
runtests_phx                 % everything
runtests_phx("noengine")     % skip MEX integration tests
runtests_phx("pure")         % headless: no engine, no graphics, no add-on toolboxes
```

Test layers are tagged `Engine` (needs the MEX), `Graphics` (needs a display-capable
session / `hgtransform`), and `Toolbox` (needs `robotics.internal.*`). Engine tests
are **assumed away** (filtered, not failed) when the MEX is absent — a green "pure"
run does not prove the engine path works.

## Writing a new simulable object

Subclass `phx.base.Object` (or `Body`/`Joint`/`Shape`) and implement the four
methods (`initObject`, `destroyObject`, static `resolveState`, static `updateView`).
Route **all** engine access through `phx.engine.io`. Start from
`phx.template.Interaction`. Avoid widening reliance on undocumented MATLAB internals
(`matlab.internal.*`, `matlab.graphics.internal.*`, `robotics.internal.*`) — the
backlog calls for isolating, not spreading, these.

## Related skills

- **phx-scene-basics** — the high-level workflow that sits on top of this gateway.
- **phx-constraints-forces** — joints/springs/thrusters and the control loop.
- **phx-logging-view** — recording and visualizing results.
- **phx-simulink** — the PhxModel co-simulation block and its property-reference ports.
