---
name: phx-upgrade-from-1x
description: >
  Convert MATLAB code, saved MAT scenes and Simulink models written for PHX 1.x
  (the Bullet-based releases 1.0.x) to PHX 2: phx.engine.BulletSettings, the
  [drag roll spin] Friction triplet, Lower/Upper Linear/AngularLimits of
  GenericJoint and BushingJoint, Resistance EnvironmentTwist/EnvironmentVelocity,
  Tube Taper, Spring/Rope Colormap names, RedrawMode "interactive", direct
  phx.engine.io calls, and the behaviour changes that need a decision
  (AppliedForce/AppliedTorque persisting, joint AppliedTorque adding to the motor,
  centre of mass of meshes and cones, engine-solved springs, Resistance rotation
  sense). Use when upgrading or porting PHX code, when old PHX code fails with
  phx:Body:frictionTriplet, an unknown property or option, or warns with
  phx:BulletSettings:replaced or phx:Resistance:renamedProperty, or when a scene
  behaves differently after the upgrade.
---

# Upgrading PHX 1.x code to PHX 2

PHX 1.x ran on Bullet Physics, PHX 2 runs on Jolt Physics. Class names and most
properties are unchanged, so most of a 1.x script runs as it is. The changes fall into
two groups, and they need different handling:

- **Syntax changes** stop the code with an error (or a deprecation warning). Rewrite
  them mechanically, by the rules below; the result behaves like the 1.x code.
- **Behaviour changes** keep the code running but move the scene differently. Do not
  rewrite them blindly: find them, decide by the rules below which reading of the 1.x
  code applies, and report every one you found and what you did.

The user-facing description of the same changes is the help page "Upgrading from PHX
1.x" (`phxdoc` → User Guide), file `phx_ug_upgrade.html` in the `doc` folder of the
toolbox. Use the other `phx-*` skills for the PHX 2 API itself.

## Workflow

1. **Inventory.** Search all `.m` and `.mlx` files of the user's project (and the
   scene sources of `.slx` models: block parameters of PHX Model, PHX Action) for the
   patterns of the two tables below. List the hits per file before you edit.
2. **Rewrite the syntax changes** (section *Syntax changes*). Keep the user's
   formatting, names and comments; change only what the rule says.
3. **Review the behaviour changes** (section *Behaviour changes*). Apply a change only
   where the rule is unambiguous; otherwise leave the code, add nothing, and put the
   question into the report.
4. **Saved scenes and Simulink** (sections below).
5. **Verify** (section *Verification*): run each converted script, with the
   deprecation warnings turned into errors, and the accuracy diagnostics on.
6. **Report** to the user: per file, the syntax rewrites made, the behaviour changes
   found with the decision taken (or the open question), and what was not verified.

Do not retune physical parameters (masses, stiffness, gains, time steps) on your own.
Where the 1.x code carries an engine workaround (see *Workarounds*), propose removing
it and let the user decide.

## Search patterns

Syntax changes (each hit must be rewritten):

| Pattern (regex) | Change |
|---|---|
| `BulletSettings` | engine settings |
| `switchEngine` | removed function |
| `phx\.engine\.io\b` | engine gateway |
| `(Lower\|Upper)(Linear\|Angular)Limits` | joint limits |
| `Friction"?\s*[,=]\s*\[` | friction triplet |
| `EnvironmentTwist\|EnvironmentVelocity\b` | Resistance names |
| `Taper` | Tube bore |
| `Colormap` (on `phx.Spring` / `phx.Rope`) | colormap matrix |
| `"interactive"` / `'interactive'` (with `RedrawMode`) | redraw mode |

Behaviour changes (each hit must be reviewed):

| Pattern (regex) | Review |
|---|---|
| `AppliedForce\|AppliedTorque` | persistent loads; joint loads with motors |
| `"(Mesh\|Extrusion\|Revolution\|Tube\|Cone)"` | centre of mass |
| `phx\.(Spring\|Rope)\b` | engine-solved springs |
| `phx\.Resistance\b` | rotation sense |
| `\.step\(` with two arguments and substeps > 1 | redraw default |
| `"kinematic"` | kinematic pose per step |
| `TorqueA\|TorqueB\|ForceA\|ForceB` | exact joint reactions |
| `applyForce\|applyTorque` with a multi-row matrix | each row a force |
| `\.mat\b` in `load`, `addObjects`, `phx.Simulation(` | saved scenes |

## Syntax changes

### Engine settings

`phx.engine.BulletSettings` is replaced by `phx.engine.Settings`. It still runs, with
the warning `phx:BulletSettings:replaced`, but will be removed. Rewrite:

- `LengthScale`, `Margin`, `MarginMode` → keep, unchanged (they exist since 1.0.16).
- `Solver` → delete. There is one solver; it solves the joints of a group of connected
  bodies together and directly (what `"dantzig"`/`"lemke"` were chosen for).
- `Iterations` → delete. Do **not** map it to `ContactIterations`: those are now caps of
  iterations that end at `ContactTolerance`/`JointTolerance` (defaults
  `ContactIterations` 20, `JointIterations` 50), and a 1.x value of 10 would lower the
  cap. Mention the deleted value in the report.
- `AutoActivated` → delete. Bodies never fall asleep.
- Properties assigned after construction (`s.Solver = ...`) → delete the same way.
- If nothing is left, drop the settings object and the `EngineSettings` argument.

```matlab
% PHX 1.x
opt = phx.engine.BulletSettings("Solver", "dantzig", "Iterations", 50, "LengthScale", 0.1);
% PHX 2
opt = phx.engine.Settings("LengthScale", 0.1);
```

`phx.engine.switchEngine(...)` → delete the call.

### Friction triplet

`Friction` was `[drag roll spin]`; it is now the Coulomb coefficient alone. A nonzero
roll or spin raises `phx:Body:frictionTriplet`.

- `[mu 0 0]` → `mu`.
- `[mu r s]` with `r` or `s` nonzero → `mu`, and **do not** convert `r`/`s`: their 1.x
  values have no equivalent. The new properties are lengths in metres, summed over
  the two bodies of a contact:
  - `RollingResistance` — arm of the normal force resisting rolling; for a body of
    radius `R` with a rolling-resistance coefficient `Crr`, `RollingResistance = Crr*R`.
  - `SpinRadius` — effective radius of the contact patch; resists spinning about the
    contact normal by `Friction*SpinRadius*N`.

  Leave both at 0 in the conversion; never pick a value yourself. Where the 1.x code
  had a nonzero roll or spin (or the comments say a body should come to rest), report it
  as an open question with the formula above, for the user to choose `Crr` (typical
  orders: a steel wheel on a rail about 0.001, a car tyre on asphalt about 0.01).
- The same applies to the `Friction` option of `phx.assembly.arena`, `chain`,
  `scatter` and `wall` (now scalar).

### Limits of GenericJoint and BushingJoint

The four 1-by-3 properties became two 3-by-2 matrices, one row `[lower upper]` per axis
X, Y, Z. The old names are not accepted (an unknown-option error). Convert axis by axis,
with `lo = Lower...(i)`, `hi = Upper...(i)`:

| 1.x | PHX 2 row `i` |
|---|---|
| `lo == hi` | `[lo hi]` (locked) |
| `lo < hi` | `[lo hi]` (bounded) |
| `lo > hi` | `[-Inf Inf]` (free; "lower > upper" is now an error) |

- An angular row must lie within (−π, π) or be `[-Inf Inf]`; a 1.x range reaching
  ±π (e.g. `[-pi pi]`) meant "practically free" → `[-Inf Inf]`.
- A linear row may have one infinite side (`[0 Inf]`).
- Defaults are unchanged in meaning: GenericJoint rigid (`zeros(3,2)`), BushingJoint
  free (`[-Inf Inf]` rows). The 1.x BushingJoint defaults `[1 1 1]`/`[-1 -1 -1]` written
  out explicitly can simply be deleted.
- Element writes: `j.LowerAngularLimits(k) = v` → `j.AngularLimits(k, 1) = v`,
  `Upper...(k)` → `(k, 2)`.

```matlab
% PHX 1.x: X rotation within +-30 deg, Z translation free
j = phx.GenericJoint(A, B, "LowerAngularLimits", [-pi/6 0 0], "UpperAngularLimits", [pi/6 0 0], ...
    "LowerLinearLimits", [0 0 1], "UpperLinearLimits", [0 0 -1]);
% PHX 2
j = phx.GenericJoint(A, B, "AngularLimits", [-pi/6 pi/6; 0 0; 0 0], ...
    "LinearLimits", [0 0; 0 0; -Inf Inf]);
```

The meaning of the angles changed too (see *Joint angles* below).

### Other renamed or removed names

- **Resistance:** `EnvironmentVelocity` → `EnvironmentLinearVelocity`,
  `EnvironmentTwist` → `EnvironmentAngularVelocity`. The old names still work with the
  warning `phx:Resistance:renamedProperty`. Rename them, and see *Resistance rotation*
  for the sign of the twist.
- **Tube `Taper`** (removed). First find the bore `d` the 1.x tube really had: the
  outer `Diameter` or `Radius`, when given, wins over `InnerDiameter`/`InnerRadius`
  (name-value pairs apply in property order, not call order) and sets
  `d = Diameter - 2*WallThickness`; otherwise `d = InnerDiameter` (or `2*InnerRadius`).
  Then, with the taper `t`, set `InnerDiameter` to `[d d*(1-t)]` for `t >= 0` or
  `[d*(1+t) d]` for `t < 0`, and **delete `Taper` and also `Diameter`, `Radius` and
  `InnerRadius`** from the call (keep `WallThickness`): a scalar outer diameter left in
  the call would set a straight bore again, silently. Check the result by building the
  shape alone: `phx.shape.Tube(...).InnerDiameter`.
- Scalar `Diameter`/`Radius` of `phx.shape.Cone` (a sharp cone, `[d 0]`) and scalar
  `InnerDiameter` of `phx.shape.Tube` (a straight bore, `[d d]`) are still valid; leave
  them.
- **Spring / Rope `Colormap`:** a name became an N-by-3 matrix. `"parula"` →
  `parula(64)` (any colormap function the same way); `"none"` → delete the option (the
  default `[]` draws `Color`). A name that is no MATLAB colormap function drew nothing
  in 1.x → delete it.
- **Simulation `RedrawMode "interactive"`** (removed) → delete the setting (the default
  `"adaptive"` replaces it).
- **`phx.engine.io(...)`** → the gateway is now `phxEngine` with a changed protocol
  (verbs, field names, one-based indices). Prefer rewriting such calls against `phx.*`
  properties and methods; if a direct call is unavoidable, use the `phx-engine-gotchas`
  skill. Never rename the call alone.

## Behaviour changes

### AppliedForce and AppliedTorque of bodies persist

In 1.x an assignment acted for **one step only**; now it acts in every step until
changed. Decide per assignment:

- Assigned before every step inside the loop → keep as it is (same result).
- Assigned once and the 1.x behaviour was a single push (the force was never meant to
  last, or the code relied on it vanishing) → replace by `body.applyForce(F)` /
  `body.applyTorque(T)` at that point, which acts for one step.
- Assigned once, but the comments or names say it is meant to be a constant load → the
  1.x scene in fact only got one step of it. Keep the assignment (it now does what was
  meant), and report that the motion will differ from 1.x.
- Unclear → leave it, and ask.

`AppliedForce` acts at the centre of mass in the body frame, and so does
`body.applyForce(F)` with a force alone (its force and point are local by default), so
the vector carries over unchanged. `applyForce` acts for one engine step, which is one
substep of `step(interval, substeps)` — as the 1.x assignment did.

### Joint loads add to the motor

`RevoluteJoint.AppliedTorque` and `PrismaticJoint.AppliedForce` replaced the motor in
1.x (a nonzero value overrode `TargetVelocity` and `MaxTorque`/`MaxForce`). Now they
act together with the motor. Where the code sets a joint load and the joint also has a
motor with nonzero `MaxTorque` (`MaxForce`), the motor now fights the load. To keep the
1.x behaviour: set `MaxTorque = 0` (`MaxForce = 0`) while the load is applied, and
restore it when the code switches back to the motor (where 1.x set the load to 0).

### Centre of mass of cones and mesh-based shapes

Every 1.x body had its centre of mass at its frame origin. Now `Body.CenterOfMass`
comes from the shape: on the axis of a `Cone` (a quarter of the height above the base),
at the mesh centroid of `Mesh`, `Extrusion`, `Revolution` and `Tube`, with `Inertia`
about it. Box, Sphere, Cylinder, Capsule, Rock and Terrain keep it at the origin.

- For each such body, run the scene setup and print `body.CenterOfMass`. If it is
  (near) zero, nothing changed.
- If it is not, the PHX 2 behaviour is the physical one; keep it by default and report
  it. Only if the user wants the 1.x motion, set `CenterOfMass = [0 0 0]`.
- Code that set `Mass`/`Inertia` by hand, computed about the origin, needs review:
  `Inertia` is now about the centre of mass.
- The mass of a `Revolution` with an open profile (an end off the axis, not returning
  to the start) changed: it is the solid closed to the axis.

### Springs and ropes are solved by the engine

1.x computed spring and rope forces in MATLAB from the previous step (explicit, with
numerical damping and a stability limit). The engine now solves them implicitly, stable
at any stiffness and damping and without numerical damping. Consequences to report:
damping added only to keep a spring stable is no longer needed, and a lightly damped
spring now oscillates longer. Keep the values unless the user decides otherwise.

### Joint angles of GenericJoint and BushingJoint

1.x limited three successive rotations X, Y, Z, with Y degenerating near ±90°. PHX 2
uses swing-twist: twist about the Z axis of the joint frame, swing about X and Y, each
limited separately, no singularity. A joint with one rotating axis behaves the same.
With several rotating axes, the limits mean something else now; report the joint. The
1.x advice "keep a free axis on X or Z, lock Y" no longer applies — a free rotation is
best on Z (twist). Do not re-orient joint frames without the user's consent.

### Resistance rotation

The environment of `phx.Resistance` rotated against the right-hand rule in 1.x; now
its velocity at `p` is `EnvironmentLinearVelocity + cross(EnvironmentAngularVelocity,
p - EnvironmentOrigin)`. To reproduce the 1.x motion, negate the value when renaming
`EnvironmentTwist` → `EnvironmentAngularVelocity`. If the comments show that the
author meant a right-hand rotation (and was surprised by 1.x), keep the sign. Report
either way.

### Smaller items

- `sim.step(interval, substeps)` redrew once at the end in 1.x (`redrawStep` defaulted
  to 0); now it animates the substeps (default 1, gated to the display rate). Pass `0`
  as the third argument only where the code depends on a single redraw (timing,
  frame capture per step).
- Kinematic bodies: a pose set once before `step(interval, substeps)` with several
  substeps is reached in the first substep, then the body stands. Where a kinematic body
  is moved by poses with substeps > 1, report it: set the pose every step without
  substeps, or drive it by `LinearVelocity`/`AngularVelocity`.
- Joint reactions (`ForceA`, `TorqueA`, `ForceB`, `TorqueB`) are exact now for all
  joint types and may differ from 1.x values for any of them; the largest known
  difference is the 1.x `TorqueA` of a revolute joint on a static or kinematic body,
  which missed the bending moment. Expected values in user code or tests may change.
- `applyForce`/`applyTorque` with several rows: each row is now a force of its own (the
  1.x resultant was wrong). No rewrite; report if results were compared to 1.x numbers.
- Contacts are solved differently (more accurate friction, less sinking, no sleeping):
  outcomes that hinge on contacts may change.

### Workarounds

Typical 1.x engine workarounds: a direct solver (`"dantzig"`, `"lemke"`), raised
`Iterations`, a much shorter step than the scene needs, extra joint or spring damping
for stability, bodies kept awake. The solver/iteration/sleep options disappear with the
settings rewrite; for step sizes and damping, propose the change and let the user
decide. The starting steps are in the `phx-scene-basics` skill and the user-guide topic
"Units, scale and engine settings".

## Saved MAT scenes

A 1.x MAT scene loads and simulates in PHX 2, and its bodies keep the centre of mass
and inertia they were saved with (a saved mesh still turns about its origin). Converted
on load: a scalar Cone `Diameter` (→ `[d 0]`), a scalar Tube `InnerDiameter` (→ `[d d]`),
`Friction [mu 0 0]` (→ `mu`). **Lost on load, silently** (left at defaults):

- GenericJoint limits (loads rigid) and BushingJoint limits (loads free);
- `Friction` with a nonzero roll or spin part (loads as 0.5, the `mu` too);
- Tube `Taper` (loads straight);
- Spring/Rope colormap names (load drawn in `Color`).

So: if the scene was built by a script (often `model_*.m` next to `saved_*.mat`),
convert the script and rebuild the MAT file from it. Otherwise read the lost values in
PHX 1.x if it is still installed (an older MATLAB release), set them again in PHX 2
after loading, and save anew. Tell the user which files were re-saved.

## Simulink

- PHX Model and PHX Action take their scene from a script, a scene function or a MAT
  file: convert those as above.
- Port references to renamed properties: `J.LowerAngularLimits(k)` →
  `J.AngularLimits(k)` and `J.UpperAngularLimits(k)` → `J.AngularLimits(k+3)` (linear
  indices of the 3-by-2 matrix, column-major); the same for the linear limits. A
  reference to the whole old property → `J.AngularLimits(1:3)` (lower) or
  `J.AngularLimits(4:6)` (upper). `Res.EnvironmentTwist` →
  `Res.EnvironmentAngularVelocity` (mind the sign), `Res.EnvironmentVelocity` →
  `Res.EnvironmentLinearVelocity`.
- `Body.AppliedForce`/`AppliedTorque` inputs: the block wrote them at every sample in
  1.x too, so nothing changes.
- The references are the mask parameters `InputRefs` and `OutputRefs` of the PHX Model
  block (a newline- or comma-separated list): edit them in the block dialog or with
  `set_param(block, "InputRefs", ...)`, then update the diagram to check that the
  ports resolve.

## Verification

Run each converted script with the deprecation warnings made errors, so that no old
name survives:

```matlab
state = warning;                                  % to restore afterwards
warning("error", "phx:BulletSettings:replaced");
warning("error", "phx:Resistance:renamedProperty");
warning("error", "phx:Shape:renamedTexture");
run("converted_script.m");
warning(state);
```

Then check the accuracy on a temporary copy of the script (never in the user's file):
right after the line that creates the `phx.Simulation`, insert
`sim.DiagnosticInfo = "warnings";` (use the script's own variable name; `"summary"` and
`"detailed"` print every step), run it, and read `getDiagnosticReport(sim)` at the end.
The warnings name the joints and contacts that did not converge, deep penetrations,
open joints and too fast bodies, each with a remedy. For results that matter, run the
copy once more with the substep halved: if the result changes, the step was too long;
propose the shorter step, do not apply it.

A clean run proves the syntax only. State in the report what was compared and what was
not. Problems that the diagnostics find but that the 1.x scene had too (bodies starting
in overlap, a part falling through the floor) are not upgrade items: list them
separately in the report, unchanged. Keep `LengthScale` and other settings that carry
over as they are.

## Related skills

- **phx-scene-basics** — the PHX 2 API of bodies, shapes, simulation and saved scenes.
- **phx-constraints-forces** — joints, limits, motors, springs and loads in PHX 2.
- **phx-engine-gotchas** — engine settings and the accuracy diagnostics used to verify.
- **phx-simulink** — port references and the PHX Model block.
