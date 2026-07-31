---
name: phx-constraints-forces
description: >
  Connect and actuate PHX bodies — joints (Fixed/Bushing/Revolute/Prismatic/
  Spherical/Gear/Generic), springs and ropes (Spring/Rope), force/field elements
  (Thruster, Buoyancy, Resistance, Dipole, Monopole) and ad-hoc
  applyForce/applyTorque. Also covers driving parameters over time with phx.Script,
  per-step control laws with phx.Function, and the read-state / set-actuator / step
  control loop. Whole jointed chains/pendulums come prefabricated from
  phx.assembly.chain. Use after phx-scene-basics when bodies must be linked, sprung,
  thrust, floated or controlled.
---

# PHX constraints & forces

All of these are `phx.base.Object` subclasses that take parent bodies as the first
positional argument(s) and then name-value pairs. Connection points (`PointA`,
`PointB`) and axes (`AxisA`, `AxisB`) are given in the **local space** of each body.

## Joints (kinematic constraints)

```matlab
A = phx.Body("Type", "static", "Position", [1 1 0], "Shape", {"Box", "SkeletPoints", [1.5 0 0]});
B = phx.Body("Position", [4 1 0], "Shape", {"Box", "SkeletPoints", [-1.5 0 0; 0 -1.5 0]});
C = phx.Body("Position", [4 -2 0], "Shape", {"Box", "SkeletPoints", [0 1.5 0]});

% Revolute: 1 DOF rotation about an axis
r = phx.RevoluteJoint(A, B, "PointA", [1.5 0 0], "PointB", [-1.5 0 0]);
r.Angle                                  % read joint angle (dependent)

% Spherical: 3 DOF (ball joint)
phx.SphericalJoint(B, C, "PointA", [0 -1.5 0], "PointB", [0 1.5 0]);
```

Joint classes:
- `phx.FixedJoint` — rigidly weld two bodies; joint frames via `TransformA/B` (4×4)
  or the dependent `PointA/B` + `EulerAnglesA/B` (z→y→x) views into them.
- `phx.BushingJoint` — compliant counterpart of `phx.FixedJoint`; an elastic bushing
  holding both bodies with per-axis `LinearStiffness/Damping` + `AngularStiffness/Damping`
  across all 6 DOF. Optional hard end stops via `Lower/UpperLinearLimits` (m) and
  `Lower/UpperAngularLimits` (rad): per axis, lower>upper = free (default), lower==upper =
  locked, lower<upper = limited travel. Very stiff springs are clamped by the solver
  (ceiling ~ mass·inertia/dt²) — add substeps or use `FixedJoint` for truly rigid.
- `phx.RevoluteJoint` — hinge; 1 rotational DOF (`PointA/B`, `AxisA/B`, dependent `Angle`).
- `phx.PrismaticJoint` — slider; 1 translational DOF along `AxisA/B`, i.e. the **local Z
  axis of the joint frame** — the same axis convention as the hinge, so the default frames
  slide along world Z. Mind that this joint also **locks the rotation about** that axis,
  which `AxisA/B` do not describe: they suffice when both bodies share an orientation, but
  otherwise set the frames in full via `TransformA/B` (4×4) or the dependent `PointA/B` +
  `EulerAnglesA/B` (z→y→x) views, so both frames coincide in *every* axis.
  See `phxex_camvalve` (valve train), `phxex_vengine`, `phxex_joints`.
- `phx.CylindricalJoint` — shaft in a sleeve; 2 DOF, translation along **and** rotation about
  `AxisA/B` (the joint frame's local Z). A prismatic and a revolute joint on one axis. Unlike
  the prismatic joint it leaves the rotation about the axis free, so `AxisA/B` alone fully
  describe it — the frames need not agree on their roll about the axis. No limits; for end
  stops use `phx.GenericJoint` with the Z axes bounded.
- `phx.SphericalJoint` — ball; 3 rotational DOF.
- `phx.GearJoint` — couples the rotation of two bodies by a ratio.
- `phx.GenericJoint` — 6 DOF, each axis of the joint frame independently locked, bounded
  or free via `Lower/UpperLinearLimits` (m) and `Lower/UpperAngularLimits` (rad): per axis,
  lower==upper = locked at that value (**all default to 0, so a bare joint is rigid** —
  open only what should move), lower<upper = bounded travel, lower>upper = free. Angles are
  right-handed (the class compensates for the engine measuring them the other way round).
  Rotation about the joint frame's **Y axis degenerates beyond ~±90°** — orient the frames
  so a freely rotating axis is X or Z. Limits are read when the simulation is built.

Axes default to local Z (`AxisA = AxisB = [0 0 1]`); set them when the hinge isn't
on Z. `Overlay` (logical) draws the joint glyph on top of geometry.

**How the frames decide the assembly.** A joint keeps the two joint frames
(`TransformA` on body A, `TransformB` on body B; `PointA`/`PointB` are their
translation parts) **in coincidence, minus the DOFs it leaves free**. So pick them
consistently with the bodies' initial poses:

- A mismatch in a **constrained** direction is not an error — the solver quietly drags
  the bodies together over the next steps (not a teleport: measured ~0.08 m in the
  first 5 ms substep, ~0.4 m after 1 s for a 0.4 m offset). That is why the hinge
  example above uses `[1.5 0 0]` and `[-1.5 0 0]` on bodies 3 m apart.
- A mismatch in a **free** direction is harmless; it only shifts the zero of that DOF.
  On a prismatic joint an offset **along** the sliding axis therefore changes nothing
  observable (`PointA = [0 0 0]` and `PointA = [0 0 -1]` behave identically for a
  vertical slider), while the components **perpendicular** to it define the line the
  body is pinned to. Only those need to be right.

Note also that a prismatic joint has **no dependent readout of its extension** (unlike
`RevoluteJoint.Angle`) — read the slide position from the body pose. Reaction loads
are available on every joint as `ForceA`/`TorqueA`/`ForceB`/`TorqueB`.

**Prefab chains:** a whole chain of jointed links (pendulum, hanging chain, rope
with collisions) is one call — `phx.assembly.chain(points, "Axis", [0 1 0],
"Anchor", "start")`. Non-zero axis rows make RevoluteJoints, zero rows (the
default) SphericalJoints; `Anchor` pins an end to a static mount ball. It returns
the links, the joints (a cell array — the classes may mix) and the anchors, all
ordinary objects you can retune. Details in **phx-scene-basics**.

**MutualCollisions** (logical, default `false`) — inherited by **all** joints (and
`phx.BushingJoint`) from `phx.base.Joint`. Bodies connected by a joint **pass through
each other by default**; pass `"MutualCollisions", true` to let them collide (e.g. a
hinged lid that must rest on its box). The flag is handed to the engine when the joint
initializes, so changing it mid-run only takes effect after a pipeline rebuild
(an add/delete of any object).

## Springs & ropes (compliant elements)

```matlab
% Spring = ideal spring + damper in parallel
s = phx.Spring(boxA, boxB, ...
    "Stiffness", 3e5, "Damping", 2e3, ...
    "FreeLength", 0, ...                   % rest length
    "PointA", [2 0 0], "PointB", [-2 0 0], ...
    "Visible", true, "Colormap", "jet", "ColorRange", [0 2e6]);  % color = force magnitude
s.Force                                    % read current force vector (3-vector)
s.Length                                   % read current length
s.Energy                                   % elastic energy (J), dependent
```

- `phx.Spring` — linear spring+damper between two points. `Colormap`/`ColorRange`
  visualize force magnitude along the drawn line.
- `phx.Rope` — a rope routed through **two or more** bodies: the first and last are
  the rope ends, every body in between acts as an ideal frictionless **pulley**, so a
  single tension acts along the whole run (a body may appear repeatedly with different
  routing points — the moving block of a tackle). Routing points are the rows of
  `Points`, in the local space of each body. The rope **transfers tension only**
  (spring+damper when stretched past its free length, slack otherwise). The free length
  is taken from the initial configuration (`InitialLength`) and shifted at runtime by
  `Displacement` — positive pays out, negative winches in, drivable by a `phx.Script`.
  `Length` and `Force` are readable/loggable. See `phxex_tackle`.

Note: `phx.BushingJoint` (the compliant 6-DOF counterpart of `phx.FixedJoint` — an
elastic bushing with per-axis linear/angular stiffness and damping) is a **joint**, not a
spring; it lives with the joints above.

## Force & field elements

### Thruster — body-fixed thrust (rocket engine / propeller / fan)

```matlab
eng = phx.Thruster(rocket, ...
    "Point", [0 0 -1.12], ...              % mount point in body-local space
    "Direction", [0 0 1], ...              % thrust direction in body-local space (unit)
    "MaxThrust", 800, ...                  % N at full throttle
    "TimeConstant", 0.2, ...               % first-order throttle lag (0 = instant)
    "ReactionFactor", 0, ...               % counter-torque per unit thrust (spinning prop)
    "ForceVectorSize", 0.004, "Color", [1 0.55 0.15]);

eng.Throttle = 0.6;     % -1..1, settable at runtime (or via phx.Script / Simulink)
eng.Direction = [g1 g2 1];   % gimbal: re-vector mid-flight
eng.Thrust              % read actual thrust after the lag (settable=private)
```

Thrust = `MaxThrust * Throttle`. `Point`/`Direction` are body-local, so the thrust
follows the body's orientation.

### Buoyancy — floating in a liquid

```matlab
wave = @(x, y, t) 0.35*sin(0.8*x - 1.6*t);     % @(x,y,t), must work elementwise

phx.Buoyancy([hull crates], ...               % one object, one shared liquid
    "Density", 1000, ...                       % kg/m^3 of the liquid
    "Level", 0, ...                            % flat level; LevelFunction overrides it
    "LevelFunction", wave, ...
    "LinearDamping", 400, "AngularDamping", 200, ... % scaled by submerged fraction
    "Resolution", 8, ...                       % sampling cells per axis (rebuild-time)
    "SurfaceSize", [14 14], "SurfaceStep", 0.25);    % drawn surface ([0 0] hides it)
```

The interior of each assigned body is sampled once by a regular grid of volume points;
the points below the surface produce the buoyant force at the centre of buoyancy, so
partially submerged bodies get the correct **righting moment** (they can also capsize).
Gotchas: the sampling sees only the **outer visual mesh**, so hollow/thin-walled shapes
behave as solid — model a boat hull as its convex hull. Open meshes
(e.g. `phx.shape.Terrain`) are unsupported. Bodies small against the grid bob between
discrete equilibria — raise `Resolution` for them. Demos: `phxex_buoyancy`,
`phxex_capsize`, `phxex_gyrostab`.

### Other elements

- `phx.Resistance` — velocity-dependent drag (aerodynamic/hydrodynamic resistance).
- `phx.Dipole` / `phx.Monopole` — magnetic/charge-like field sources; bodies carrying
  them attract/repel (see `phxex_magnets`, `phxex_charges`, `phxex_maglev`).

### Ad-hoc forces — `Body.applyForce` / `applyTorque`

For actuation that isn't a standing element, push straight on the body. Both act
during **one subsequent step only** and are reset afterwards, so call them every step
(from a `phx.Function` or your control loop):

```matlab
body.applyForce(F);                              % at the body origin, F in LOCAL space
body.applyForce(F, p);                           % at local point p
body.applyForce(F, p, isLocalForce, isLocalPoint);   % both default TRUE
body.applyTorque(T);                             % T in local space
body.applyTorque(T, false);                      % ... or in world space
```

Mind the two independent frame flags: `isLocalForce` rotates the vector with the body,
`isLocalPoint` interprets the point of application in body coordinates. Passing a world
force at a local point (or vice versa) is legal and often what you want — e.g. gravity-
like world force applied at a local hardpoint. `applyTorque` is the usual way to drive a
wheel or a shaft, since PHX joints carry no motors.

## Driving parameters over time — phx.Script

`phx.Script` sets parameters of other objects during the simulation, either from an
interpolated curve or a time-callback. Each curve is a cell:
`{'ParamName', timeVector, valuesVector, 'interp', 'extrap'}` **or**
`{'ParamName', 't-expression-string'}` (compiled to `@(t) ...`). The `interp`/`extrap`
strings are `interp1` method names (`'linear'`, `'nearest'`, `'pchip'`, …; default
`'linear'`), or `'repeat'` as the extrap to loop the curve — **not** `'hold'`. The
target parameter and the curve values must have matching size (a scalar curve drives
a scalar property like `Throttle`, not a 3-vector like `Direction`).

```matlab
% Numeric curve: ramp the throttle 0 -> 1 over 1 s, hold, then back to 0 at 5 s
phx.Script(eng, {'Throttle', [0 1 3 5], [0 1 1 0], 'linear', 'linear'});

% Callback form: the string is compiled to @(t) ...
phx.Script(eng, {'Throttle', '0.5 + 0.5*sin(t)'});
```

The script runs `before` the physics each step, so the parameter is fresh for that step.

## Custom per-step logic — phx.Function

`phx.Function` inserts an arbitrary callback into the simulation pipeline without
writing a `phx.base.Object` subclass — the middle ground between `phx.Script`
(declarative, feedforward) and a full custom class. Use it for control laws, custom
force laws and filters.

```matlab
% Bound to sensor + actuator objects; runs before the physics of every (sub)step
phx.Function({ball, plate}, @(o, p, dt, t) balanceLaw(p, Kp, Kd, g, maxTilt));
```

Signature `fcn(obj, parents, dt, time)`: `obj` is the `phx.Function` itself (keep
integrator state / previous samples in its `UserData`), `parents` the bound objects
(cell array, same as `obj.Parents`), `dt` the **substep** size and `time` the
simulation time. There is no return value — the callback acts by writing to the bound
objects (set `Throttle`, call `applyTorque`, move a kinematic body, …).

**Prefer this over a hand-rolled `while` loop** when the logic must run at the physics
rate: it fires on every substep, so you can keep `sim.step(interval, substeps, …)` with
many substeps instead of stepping one substep at a time from MATLAB. Demos:
`phxex_balance` (PD ball-on-plate), `phxex_drone`.

## Closed-loop / interactive control pattern

When control logic must react to state *and* drive the MATLAB-side loop (HUD updates,
early termination, per-frame logging), step in a loop and read/write dependent
properties between steps. This is the canonical PHX controller shape (from
`phxex_rocket`); for a pure control law that only needs to run at the physics rate,
`phx.Function` above is simpler and faster:

```matlab
sim = phx.Simulation(ax);
dt = 0.005; subSteps = 10;
t = 0;
while t < tMax
    for s = 1:subSteps
        p = rocket.Position;          % read live engine state
        v = rocket.LinearVelocity;
        R = rocket.Transform(1:3, 1:3);

        % ... your controller computes a command ...
        eng.Throttle  = throttleCmd;  % write the actuator
        eng.Direction = gimbalCmd;

        sim.step(dt, 1, 1);           % advance ONE substep
        t = t + dt;
    end
    % per-frame logging / HUD update here
end
delete(sim);
```

`sim.step` renders on its own here too — no `drawnow`/`pause(0)` belongs in this loop
unless you draw your own overlay or need the figure to stay clickable; see
**phx-scene-basics**.

A controlled vehicle that momentarily hovers near-still keeps responding to thrust
because Bullet sleeping is off by default (`BulletSettings.AutoActivated=false`);
see **phx-engine-gotchas**.

Keep `dt ≤ 5 ms` for tight constraint networks (joint chains, stacked contacts) or
they destabilize. You can create or delete joints/springs mid-run freely:
`delete(obj)` removes the engine constraint immediately and rebuilds the simulation
pipelines automatically; add new elements via `sim.addObjects(...)`. See
**phx-engine-gotchas**.

## Related skills

- **phx-scene-basics** — bodies, shapes, `Simulation.step`, running headless.
- **phx-logging-view** — record forces/angles/velocities with `phx.Logger`, draw
  `phx.Trace`, detect bodies in a region with `phx.Zone`.
- **phx-engine-gotchas** — pipeline rebuilds, `dt` stability, `BulletSettings`, error IDs.
- **phx-simulink** — drive actuator/sensor properties from Simulink (the co-simulation block).
