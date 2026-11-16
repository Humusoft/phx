---
name: phx-constraints-forces
description: >
  Connect and actuate PHX bodies — joints (Fixed/Bushing/Revolute/Prismatic/
  Cylindrical/Spherical/Gear/Generic) and their end stops (limits, swing cone), springs and
  ropes (Spring/Rope/TorsionSpring), the persistent body loads
  AppliedForce/AppliedTorque, force/field elements
  (Thruster, Buoyancy, Resistance, Dipole, Monopole) and one-step
  applyForce/applyTorque. Also covers driving parameters over time with phx.Script,
  per-step control laws with phx.Function, and the read-state / set-actuator / step
  control loop. Whole jointed chains/pendulums come prefabricated from
  phx.assembly.chain. Use after phx-scene-basics when bodies must be linked, sprung,
  thrust, floated or controlled.
---

# PHX constraints and forces

All of these are `phx.base.Object` subclasses that take parent bodies as the first
positional argument(s) and then name-value pairs. Connection points (`PointA`,
`PointB`) and axes (`AxisA`, `AxisB`) are given in the **local space** of each body.

Sections: **Joints**, **Joint motors** (with the joint loads), **Springs and ropes**,
**Force and field elements** (body loads, thrusters, buoyancy, drag, fields, one-step
forces), then time-driven parameters (**phx.Script**), per-step logic
(**phx.Function**) and **closed-loop and interactive control**. To drive a joint use its
motor or joint load; to push a free body, its body loads.

## Joints (kinematic constraints)

```matlab
A = phx.Body("Type", "static", "Position", [1 1 0], "Shape", {"Box"});
B = phx.Body("Position", [4 1 0], "Shape", {"Box"});
C = phx.Body("Position", [4 -2 0], "Shape", {"Box"});

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
  across all 6 DOF. Optional hard end stops via `LinearLimits` (m) and `AngularLimits`
  (rad), 3×2, one row `[lower upper]` per axis X, Y, Z; default `[-Inf Inf]` in every row
  (spring only), `[v v]` locks the axis rigidly, a range limits the travel while the spring
  acts inside it. Springs and limits can be changed at any time, also mid-run. The springs
  are solved implicitly: stable at any stiffness (a very stiff one holds almost like
  `FixedJoint`), with a little numerical damping at coarse steps.
- `phx.RevoluteJoint` — hinge; 1 rotational DOF (`PointA/B`, `AxisA/B`). **Motorized:**
  `TargetVelocity` (rad/s) + `MaxTorque` (N·m), or a plain torque `AppliedTorque` (N·m)
  — see the motor section below. End stops: `Limits` (rad), e.g. `[0 pi/2]` for a door.
- `phx.PrismaticJoint` — slider; 1 translational DOF along `AxisA/B`, i.e. the **local Z
  axis of the joint frame** — the same axis convention as the hinge, so the default frames
  slide along world Z. Mind that this joint also **locks the rotation about** that axis,
  which `AxisA/B` do not describe: they suffice when both bodies share an orientation, but
  otherwise set the frames in full via `TransformA/B` (4×4) or the dependent `PointA/B` +
  `EulerAnglesA/B` (z→y→x) views, so both frames coincide in *every* axis.
  **Motorized:** `TargetVelocity` (m/s) + `MaxForce` (N), or a plain force `AppliedForce` (N).
  End stops: `Limits` (m), one side may be open (`[-Inf 0.5]`).
  See `phxex_camvalve` (valve train), `phxex_cartpendulum`.
- `phx.CylindricalJoint` — shaft in a sleeve; 2 DOF, translation along **and** rotation about
  `AxisA/B` (the joint frame's local Z). A prismatic and a revolute joint on one axis. Unlike
  the prismatic joint it leaves the rotation about the axis free, so `AxisA/B` alone fully
  describe it — the frames need not agree on their roll about the axis. `Angle` and
  `Distance` report both of its coordinates. End stops: `LinearLimits` (m) and
  `AngularLimits` (rad), each 1×2. It has **no motor**. See `phxex_vengine` (pistons and
  wrist pins).
- `phx.SphericalJoint` — ball; 3 rotational DOF. Optional limits for shoulders and hips:
  `ConeAngle = [angleX angleY]` (rad, default `[pi pi]` = free) bounds the swing of frame
  B's Z axis by an elliptic cone about frame A's Z axis (half-angles about A's X and Y; a
  scalar is a circular cone, 0 locks that direction); `TwistLimits` (rad) bounds the twist
  about Z. With limits the **joint frames matter** (`AxisA/B`, `TransformA/B`): Z is the
  cone and twist axis. `"TwistLimits", 0` with a free cone is a **constant-velocity
  (homokinetic) joint** — B turns about its axis by exactly A's angle at any shaft angle.
- `phx.GearJoint` — couples the rotation of two bodies by a ratio.
- `phx.GenericJoint` — 6 DOF, each axis of the joint frame independently locked, bounded
  or free via `LinearLimits` (m) and `AngularLimits` (rad), each 3×2 with one row
  `[lower upper]` per axis X, Y, Z: `[v v]` = locked at v (**the default is `zeros(3,2)`,
  so a bare joint is rigid** — open only what should move), a range = bounded travel,
  `[-Inf Inf]` = free. E.g. a slider on x bounded to ±1 m:
  `"LinearLimits", [-1 1; 0 0; 0 0]`. Angles are right-handed. The rotation is split into
  a **twist** about the frame's Z axis and a **swing** about X and Y, each limited
  separately; give a freely rotating axis Z (the twist), as `CylindricalJoint` does.

**End stops — one set of rules for every joint.** A limit is a pair `[lower upper]` of the
joint coordinate (`Angle` rad, `Distance` m); equal values or a scalar lock the joint
there, `[-Inf Inf]` (the default except `GenericJoint`) leaves it free, a linear pair may
open one side with `-Inf`/`Inf`. Angular pairs are both within (-π, π) or `[-Inf Inf]` —
the angle wraps, so there are no one-sided angular stops. Reversed or NaN limits error with
`phx:Joint:invalidLimits`. Stops are hard and do not bounce; their reaction is in
`ForceA`…`TorqueB`, so a body resting on a stop reports its weight there (it chatters
step to step: average over a few steps). All limits can be changed mid-run or driven from
Simulink — move a stop, lock a joint and release it again.

Axes default to local Z (`AxisA = AxisB = [0 0 1]`); set them when the hinge isn't
on Z. `Overlay` (logical) draws the joint glyph on top of geometry.

**How the frames decide the assembly.** A joint keeps the two joint frames
(`TransformA` on body A, `TransformB` on body B; `PointA`/`PointB` are their
translation parts) **in coincidence, minus the DOFs it leaves free**. So pick them
consistently with the bodies' initial poses:

- A mismatch in a **constrained** direction is not an error, but the solver snaps the
  bodies together at once, violently: a hinge whose frames were 0.4 m apart moved its
  arm the whole 0.4 m in the first 5 ms substep (the gap then 0.16 m as the arm swung,
  closed within 1 s). So make the frames coincide in the
  initial poses; that is why the hinge example above uses `[1.5 0 0]` and `[-1.5 0 0]`
  on bodies 3 m apart.
- A mismatch in a **free** direction is harmless; it only shifts the zero of that DOF.
  On a prismatic joint an offset **along** the sliding axis therefore changes nothing
  observable (`PointA = [0 0 0]` and `PointA = [0 0 -1]` behave identically for a
  vertical slider), while the components **perpendicular** to it define the line the
  body is pinned to. Only those need to be right.

**Joint coordinates.** `phx.base.Joint` gives *every* joint two read-only dependent
properties: `Angle` (rad) is the rotation of the second joint frame about the joint axis
and `Distance` (m) its offset along that axis, both measured from the first frame. They
are worked out from the frames, so they read the pose the bodies hold right now and need
no simulation. Where the joint leaves that DOF free they are the joint coordinate itself
(hinge angle, slider travel, both at once on a cylindrical joint); where it locks it they
sit near zero and show the error the solver leaves. `Angle` is a **projection onto the
axis**, not the shortest arc, so on a `SphericalJoint` it reports only the turn about the
axis and ignores the tilt; it wraps to ±π and does not accumulate, so unwrap it yourself
(`unwrap` in MATLAB, the DSP `Unwrap` block with `running = on` in Simulink) when a joint
spins through more than half a turn. Reaction loads are available on every joint as
`ForceA`/`TorqueA`/`ForceB`/`TorqueB`.

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

## Joint motors — phx.RevoluteJoint, phx.PrismaticJoint

Both joints can drive themselves. The motor is a **saturated velocity source**: it holds
`TargetVelocity` while it needs less than `MaxTorque` (N·m) / `MaxForce` (N), and delivers
exactly that limit when the target is out of reach. Two numbers therefore cover three kinds
of drive:

| To get | TargetVelocity | MaxTorque / MaxForce |
|---|---|---|
| no motor (default) | anything | `0` |
| speed drive | the wanted speed, signed | the limit |
| friction brake | `0` | the holding torque/force |

```matlab
hub = phx.RevoluteJoint(base, disc, "TargetVelocity", 6, "MaxTorque", 5);
sim.step(1, 100);               % spins up to 6 rad/s and holds it
hub.TargetVelocity = 0;         % same motor, now a brake
hub.MaxTorque = 1;              % lets go once the load passes 1 N*m
```

- Both properties are plain settable doubles: change them mid-run, drive them from a
  `phx.Script` curve or a `phx.Function` control law, or wire them to Simulink ports.
- The limits are **physical units**, not fractions — a cubic metre of default density
  already weighs a tonne, so size them against the real load. `0` means no motor at all.
- The motor is part of the constraint, so its **reaction lands on the other body**. Two
  free bodies both turn, in opposite directions, split inversely by their inertias; the
  motor sets their *relative* velocity, not the absolute speed of either.
- Position control is not built in. Do it yourself by writing `TargetVelocity` each step
  from the `Angle`/`Distance` error — that is the one mode that costs a per-step write.
- `phx.CylindricalJoint` has no motor, and a prismatic joint's rotation is locked, so its
  slider has nothing to turn.

### Joint loads — `RevoluteJoint.AppliedTorque`, `PrismaticJoint.AppliedForce`

When the drive is a torque or a force rather than a speed (an actuator model, a
hydraulic cylinder computed in Simscape, a control law that outputs N·m), set
`RevoluteJoint.AppliedTorque` (N·m) or `PrismaticJoint.AppliedForce` (N). It acts on
body B about/along the joint axis and, reversed, on body A, and it **stays until you
change it** — no per-step call, unlike `applyTorque`.

```matlab
hub = phx.RevoluteJoint(base, disc, "AppliedTorque", 2);   % 2 N*m from now on
sim = phx.Simulation(gca);
sim.step(1, 100);
hub.AppliedTorque = 0;          % off again
```

- It is independent of the motor: both act at once, the motor regulating against the
  applied value like against any other load (a speed drive keeps its speed under an applied
  torque as long as `MaxTorque` allows). For a pure torque/force source leave the motor off.
- Like the motor it acts between the two bodies, so its reaction lands on body A, and it
  shows in the joint's `ForceA`…`TorqueB` feedback.
- Signed; the sign follows the joint axis like `Angle` / `Distance`.

## Springs and ropes (compliant elements)

```matlab
% Spring = ideal spring + damper (+ inerter) in parallel
s = phx.Spring(boxA, boxB, ...
    "Stiffness", 3e5, "Damping", 2e3, ...
    "FreeLength", 0, ...                   % rest length
    "Inertance", 0, ...                    % kg; an inerter adds it to the moving mass
    "PointA", [2 0 0], "PointB", [-2 0 0], ...
    "Visible", true, "Colormap", jet(64), "ColorRange", [0 2e6]);  % color = force magnitude
s.Force                                    % force on boxA over the last step (3-vector)
s.Length                                   % current length (distance of the points)
s.Energy                                   % elastic energy (J), dependent

% Torsion spring about the z axis of TransformA (frames as for the joints)
t = phx.TorsionSpring(frame, door, "TransformB", T, ...
    "Stiffness", 50, "Damping", 10, "FreeAngle", 0);   % N*m/rad, N*m*s/rad, rad
t.Angle                                    % continuous over turns while running
t.Torque                                   % torque on the first body (3-vector)
```

- **All three are constraints of the engine**, solved with the joints and contacts by a
  step that keeps their energy and is **stable at any stiffness and damping** — a stiff
  spring needs no shorter `dt` than the rest of the scene. They need no MATLAB work per
  step. Changing `Stiffness`, `Damping`, `FreeLength` or `Inertance` mid-run takes effect
  at once.
- `phx.Spring` — linear spring+damper between two points, force
  `k·(L − FreeLength) + c·L' + b·L''`. `Inertance` b (kg) is an ideal inerter in parallel:
  a mass m on it oscillates as m + b (suspension and vibration-isolation models).
  `Length` is the full current length (extension = `Length - FreeLength`).
  `Colormap` (an N×3 matrix such as `jet(64)`, not a name; empty = the fixed `Color`, which
  the colormap leaves unchanged) and `ColorRange` visualize force magnitude along the drawn line.
- `phx.TorsionSpring` — torsion spring+rotary damper (+ rotary `Inertance`, kg·m²) about
  the z axis of `TransformA`; it only turns the bodies about that axis, so put it on a
  `phx.RevoluteJoint` with the **same frames** to spring the hinge. The angle (that of
  the joints) is followed continuously, so it winds over any number of turns like a clock
  spring.
- `phx.Rope` — a rope routed through **two or more** bodies: the first and last are
  the rope ends, every body in between acts as an ideal frictionless **pulley**, so a
  single tension acts along the whole run (a body may appear repeatedly with different
  routing points — the moving block of a tackle). Routing points are the rows of
  `Points`, in the local space of each body. The rope **transfers tension only**
  (spring+damper when stretched past its free length, slack otherwise). The free length
  is taken from the initial configuration (`InitialLength`) and shifted at runtime by
  `Displacement` — positive pays out, negative winches in, drivable by a `phx.Script`.
  `Length` and `Force` (the tension) are readable/loggable. It needs at least two
  **different** bodies. See `phxex_tackle`.

Note: `phx.BushingJoint` (the compliant 6-DOF counterpart of `phx.FixedJoint` — an
elastic bushing with per-axis linear/angular stiffness and damping) is a **joint**, not a
spring; it lives with the joints above.

## Force and field elements

### Body loads — `Body.AppliedForce` / `AppliedTorque`

The plain way to push a body: a force at its **centre of mass** and a torque, both in
the **body frame**. The engine keeps them and adds them in **every step, turning with
the body, until changed** — no MATLAB work per step.

```matlab
body.AppliedForce  = [0 0 900];   % N at the centre of mass, along the body z axis
body.AppliedTorque = [0 0 2];     % N*m about the body z axis
body.AppliedForce  = [0 0 0];     % off again
```

- **Force at a point off the centre:** force `F` at body-frame point `r` = `AppliedForce`
  `F` plus `AppliedTorque` `cross(r, F)`; several such forces simply add up. `r` is taken
  from the centre of mass: for a point `p` relative to the body origin, `r = p -
  body.CenterOfMass` (zero for box, sphere, cylinder, capsule; a cone or a mesh has its
  centre elsewhere).
- **World-fixed direction** (e.g. wind): rotate into the body frame yourself, or use
  `applyForce(F, p, false, ...)` every step.
- **Varying in time:** write the properties from a `phx.Function`, from your control
  loop or from a Simulink input port (`Body.AppliedForce(3)`); in Simulink the **PHX Point
  Forces** block sums forces at several points into the two inputs (see phx-simulink).
- A static or kinematic body takes no load.

Reach for `phx.Thruster` (below) only when you need what it adds on top: one `Throttle`
scaled by `MaxThrust`, a response lag, the propeller reaction torque, the drawn exhaust,
`Thrust` for a `phx.Logger`. A thruster is computed in MATLAB at every substep (several
µs each), the body loads cost nothing.

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
follows the body's orientation. Several thrusters on one body add up (`phxex_drone`
has four, `phxex_rocket` one with a gimballed `Direction`).

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
`phxex_gyrostab`.

### Drag and fields — Resistance, Monopole, Dipole

**`phx.Resistance` — drag against a moving environment** (air, water, wind). One object
serves all its bodies:

```matlab
air = phx.Resistance([ball crate], ...
    "VelocityFactors", [0 0.5 0.2], ...               % F = f1 + f2*|v| + f3*v^2 (N), per axis
    "EnvironmentLinearVelocity", [3 0 0], ...          % wind, m/s
    "EnvironmentAngularVelocity", [0 0 0.1], ...       % rad/s about EnvironmentOrigin
    "EnvironmentOrigin", [0 0 0]);
```

`v` is the body's velocity relative to the environment (which moves at
`EnvironmentLinearVelocity + cross(EnvironmentAngularVelocity, p - EnvironmentOrigin)`, right-hand
rule). The force acts at the centre of mass against `v`, computed **per world axis**
(`f(k)*|v_i|^(k-1)` in each component), so it is no exact quadratic drag on a diagonal
motion, and it does not slow rotation. The default `VelocityFactors` 0 does nothing.

**`phx.Monopole` / `phx.Dipole` — pairwise fields** (charges, magnets, gravity). Every
pair of bodies of one object attracts or repels by `F = Attractivity*c1*c2/r^2`; with the
default `Attractivity` -1 opposite charges attract.

```matlab
phx.Monopole(cubes, "Charge", [1 1 -1 -1]*1e4);                 % one Charge per body (or a scalar)
phx.Monopole(planets, "Attractivity", 6.67430e-11, ...           % gravity: masses as charges
    "Charge", [planets.Mass], "Visible", false);
phx.Dipole(magnets, "Charge", 1, "Axis", [0.05 0 0]);            % poles at +-Axis (body frame)
```

A `Monopole` acts between the body origins; a `Dipole` puts `+Charge` and `-Charge` at
`+Axis` and `-Axis` of each body, so it also turns the bodies (magnets align). The force
has no softening: it grows without bound as `r` → 0 and is not finite for coincident
poles, so keep charged bodies apart (collisions on). The `VectorField*` properties set
the drawn field arrows, `Visible` hides them; `Energy` is the potential energy of the
pairs. Demos: Monopole `phxex_charges`, `phxex_gravity`, `phxex_maglev`,
`phxex_capacitor`; Dipole `phxex_magnets`, `phxex_magpendulum`.

**Moving surfaces.** Not an element but the same job: to drive bodies through contact
(conveyor belts, roller tracks, a self-propelled vehicle), give a body the
`SurfaceVelocity` property rather than scripting its pose — see phx-scene-basics and
`phxex_beltmaze`.

### One-step forces — `Body.applyForce` / `applyTorque`

For a kick, or a force that needs a world-frame direction or point, push straight on
the body. Both act during **one engine step only** (one substep of `sim.step`: called
before `sim.step(T, n)` they act for `T/n`) and are reset afterwards, so call them every
substep (from a `phx.Function`) to keep them acting:

```matlab
body.applyForce(F);                              % at the centre of mass, F in LOCAL space
body.applyForce(F, p);                           % at local point p
body.applyForce(F, p, isLocalForce, isLocalPoint);   % both default TRUE
body.applyTorque(T);                             % T in local space
body.applyTorque(T, false);                      % ... or in world space
```

For a load that should stay, set the body loads above instead.

Mind the two independent frame flags: `isLocalForce` rotates the vector with the body,
`isLocalPoint` interprets the point of application in body coordinates. Passing a world
force at a local point (or vice versa) is legal and often what you want — e.g. gravity-
like world force applied at a local hardpoint. To drive a wheel or a shaft use the
**joint motor** or the joint's `AppliedTorque` above, not `applyTorque` — an external
torque on a jointed body puts no reaction on the body it is hinged to, so the chassis
never feels the push it should.

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
objects (set `AppliedForce` or `Throttle`, call `applyTorque`, move a kinematic body, …).

**Prefer this over a hand-rolled `while` loop** when the logic must run at the physics
rate: it fires on every substep, so you can keep `sim.step(interval, substeps)` with
many substeps instead of stepping one substep at a time from MATLAB. Demos:
`phxex_balance` (PD ball-on-plate), `phxex_drone`.

## Closed-loop and interactive control

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

        sim.step(dt);                 % advance ONE substep
        t = t + dt;
    end
    % per-frame logging / HUD update here
end
delete(sim);
```

`sim.step` renders on its own here too — no `drawnow`/`pause(0)` belongs in this loop, not
even for your own overlay (the next redraw shows it); keeping the figure clickable and
thinning the frames is already the default `RedrawMode`'s job. See **phx-scene-basics**.

A controlled vehicle that momentarily hovers near-still keeps responding to thrust
because bodies never fall asleep; see **phx-engine-gotchas**.

The step size for joints and contacts: **phx-scene-basics**, "Practical tips". Joints and
springs can be created or deleted mid-run (`sim.addObjects`, `delete`); see
**phx-engine-gotchas**, "Known quirks".

## Related skills

- **phx-scene-basics** — bodies, shapes, `Simulation.step`, running headless.
- **phx-logging-view** — record forces/angles/velocities with `phx.Logger`, draw
  `phx.Trace`, detect bodies in a region with `phx.Zone`.
- **phx-engine-gotchas** — pipeline rebuilds, `phx.engine.Settings`, accuracy diagnostics, error IDs.
- **phx-simulink** — drive actuator/sensor properties from Simulink (the co-simulation block).
- **phx-upgrade-from-1x** — converting code, saved scenes and Simulink models written for PHX 1.x.
