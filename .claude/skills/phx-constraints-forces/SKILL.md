---
name: phx-constraints-forces
description: >
  Connect and actuate PHX bodies — joints (Fixed/Revolute/Spherical/Gear/Generic),
  springs and ropes (Spring/GenericSpring/Rope), and force/field elements
  (Thruster, Resistance, Dipole, Monopole). Also covers driving parameters over
  time with phx.Script and the read-state / set-actuator / step control loop.
  Whole jointed chains/pendulums come prefabricated from phx.assembly.chain. Use
  after phx-scene-basics when bodies must be linked, sprung, thrust, or controlled.
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
- `phx.RevoluteJoint` — hinge; 1 rotational DOF (`PointA/B`, `AxisA/B`, dependent `Angle`).
- `phx.SphericalJoint` — ball; 3 rotational DOF.
- `phx.GearJoint` — couples the rotation of two bodies by a ratio.
- `phx.GenericJoint` — configurable per-axis `LinearLimits`/`AngularLimits` for custom
  constraints. **Marked UNDER DEVELOPMENT in the class — use with caution.**

Axes default to local Z (`AxisA = AxisB = [0 0 1]`); set them when the hinge isn't
on Z. `Overlay` (logical) draws the joint glyph on top of geometry.

**Prefab chains:** a whole chain of jointed links (pendulum, hanging chain, rope
with collisions) is one call — `phx.assembly.chain(points, "Axis", [0 1 0],
"Anchor", "start")`. Non-zero axis rows make RevoluteJoints, zero rows (the
default) SphericalJoints; `Anchor` pins an end to a static mount ball. It returns
the links, the joints (a cell array — the classes may mix) and the anchors, all
ordinary objects you can retune. Details in **phx-scene-basics**.

**MutualCollisions** (logical, default `false`) — inherited by **all** joints (and
`phx.GenericSpring`) from `phx.base.Joint`. Bodies connected by a joint **pass through
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
- `phx.GenericSpring` — 6-DOF compliant constraint with per-axis limits. **Marked
  UNDER DEVELOPMENT in the class — use with caution.**
- `phx.Rope` — an inextensible/limited rope link between bodies.

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

### Other elements

- `phx.Resistance` — velocity-dependent drag (aerodynamic/hydrodynamic resistance).
- `phx.Dipole` / `phx.Monopole` — magnetic/charge-like field sources; bodies carrying
  them attract/repel (see `phxex_magnets`, `phxex_charges`, `phxex_maglev`).

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

## Closed-loop / interactive control pattern

When control logic must react to state, step in a loop and read/write dependent
properties between steps. This is the canonical PHX controller shape (from `phxex_rocket`):

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
- **phx-logging-view** — record forces/angles/velocities with `phx.Logger`, draw `phx.Trace`.
- **phx-engine-gotchas** — pipeline rebuilds, `dt` stability, `BulletSettings`, error IDs.
- **phx-simulink** — drive actuator/sensor properties from Simulink (the co-simulation block).
