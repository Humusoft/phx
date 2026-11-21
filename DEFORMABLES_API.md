# Points of deformable bodies and particle groups: proposal (2026-11-21)

Status: agreed in discussion and implemented in six phases (all done 2026-11-21, see
Phases). Covers `phx.SoftBody`, `phx.FlexibleBody`,
`phx.Particles` and `phx.Liquid` (with `phx.base.ParticleGroup`). Follows the mass
properties work (MASS_API.md), which settled the shape density.

## Why

The four classes are built from points, but name and place them differently:

| | SoftBody, FlexibleBody (2.0.0) | Particles (2.0.0), Liquid (new) |
|---|---|---|
| points | `Vertices`, body frame, the rest shape | `Positions`, world frame |
| present points | `CurrentVertices`, read-only | `Positions`, read/write |
| placement | `Position`, `Orientation` of the rest shape | `Position`, `Orientation` of the filled `Shape` only |
| velocities | none | `Velocities` |
| damping | `Damping` | `LinearDamping` |

`Position` beside `Positions` on one object is confusing (one letter between moving the
group and overwriting every point); generic code needs a different property per class
for the present points; a deformable body cannot be given a velocity at all.

## The model

| | SoftBody, FlexibleBody | Particles, Liquid |
|---|---|---|
| `Vertices` (N×3, body frame) | the **rest shape**: the definition every build of the engine body starts from; the run never changes it | the **live points** in the frame of the group: before a simulation the starting layout, during it read and written in the engine, after it the last state, from which the next simulation goes on |
| `CurrentVertices` (N×3, world, read-only) | during the run from the engine; after it the last state (kept when the simulation ends); before the first one the rest shape at `Position`/`Orientation` | `Vertices` placed at `Position`/`Orientation` |
| `Velocities` (N×3, world) | the **initial** vertex velocities (definition, like `Vertices`); zeros by default | the **live** velocities, as `Vertices` |
| `CurrentVelocities` (N×3, world, read-only) | during the run from the engine, after it the last state, before it `Velocities` | = `Velocities` |
| `Position`, `Orientation` (and `AxisAngle`, `EulerAngles`, `Quaternion`) | where the rest shape is placed when the body is built | where the group is placed |
| `Shape` | sets `Vertices` (and `Faces` / `Tetrahedra`) | sets `Vertices`: the lattice of the shape, in its frame |

- **Why two properties for meshes and one for particles.** The rest shape is what the
  elasticity refers to, and PHX rebuilds the engine body from it at every new simulation
  (`delete(sim)`, every Simulink run, a scene loaded from a MAT file); a deformed state
  taken as the new rest shape would silently drop the stress. Particles have no rest
  shape: their starting layout and their state are one, and a new simulation already
  goes on from the last state (`tParticles/stateSurvivesTheSimulation`).
- **No per-step cost.** The engine is read when a property is read, and once when the
  simulation ends: `phx.Simulation.destroyObject` calls `stateTransfer` on every object
  before it clears the world. There the particle groups write their last state into
  `Vertices`/`Velocities` (in the group frame), and the meshes keep their last
  vertices and velocities for `CurrentVertices`/`CurrentVelocities` (today they read
  back the rest shape while the drawing shows the deformed one).
- `Velocities` accept a single row 1×3 for all points, as `Inertia` accepts a scalar;
  otherwise one row per point.
- World frame for velocities, as `phx.Body.LinearVelocity`.
- `CurrentVertices` and `CurrentVelocities` exist on all four classes, so that code
  reading the present points of any of them needs one name.

### Particle groups in detail

- `Count` stays. `add(P [, V])` and `remove(idx)` work in world coordinates on the
  present points, also during a run; before one, `add` converts to the group frame.
- `Position`/`Orientation` place the group when it is built (as for meshes); the frame
  does not move during the run.
- **The shape is kept and the lattice filled again** when `Spacing` (Liquid) or
  `Radius` (Particles) changes, until the simulation builds the group, as today, but no
  longer when `Position`/`Orientation` change: the lattice is in the group frame. The
  meshes convert their shape at once and do not keep it: their fineness is the shape's
  own `ElementResolution`, while the spacing of the lattice belongs to the group (and
  the mass of a liquid particle follows from it).

## Damping

`SoftBody.Damping` and `FlexibleBody.Damping` become `LinearDamping`, the name the
particle groups use for the same quantity (dv/dt = -c v). `Damping` stays as a
deprecated alias with the warning `phx:SoftBody:renamedProperty` /
`phx:FlexibleBody:renamedProperty`, as `phx.Resistance` did.

## Restitution of flexible bodies

Not added: the Newton solver of flexible bodies has no restitution, its contacts do not
bounce. The help of `phx.FlexibleBody` says so. A bouncing contact in the implicit solver
would be a change of Jolt (an item of JOLT_CHANGES.md) of its own.

## Engine (tools/jolt)

- Soft bodies: a `velocities` field (set and get, 3×N, world) on the vertices of the
  Jolt soft body.
- Flexible bodies: the Newmark solver keeps the node velocities in its own state
  (`FlexibleSolver::mV`, with its history), not in the soft body vertices: a setter
  there, at the start of a step, is a change of our flexible solver in the Jolt clone
  (an item of JOLT_CHANGES.md).
- Particle groups have `positions` and `velocities` already (PROTOCOL.md section 7).

## Compatibility

- `Particles.Positions` (2.0.0) stays as a deprecated alias with the warning
  `phx:Particles:renamedProperty`: reading gives `CurrentVertices`, writing sets the
  points from world coordinates. With the default `Position` [0 0 0] and `Orientation`
  eye(3), the world and the group frame agree, so `phx.Particles(ax, "Positions", P)`
  keeps working unchanged.
- `Damping` of soft and flexible bodies: deprecated alias (above).
- `phx.Liquid`, `Shape`/`Position`/`Orientation` of particle groups and
  `ParticleGroup` are newer than 2.0.0: no alias.
- Saved scenes: a 2.0.0 `phx.Particles` has its points in world coordinates and no
  `Position`, so they load as its `Vertices`. Meshes load as before.
- Demos to move: `phxex_balancewater`, `phxex_plow`, `phxex_soilparticles` (Positions),
  `phxex_cantilever` and the others with `Damping` on a deformable body; tests `tLiquid`,
  `tParticles`, `tSoftBodies`.

## Tests

- Each class: `Vertices` and `CurrentVertices` before, during and after a run (meshes:
  the rest shape kept, the last state read after the run; particles: the last state in
  `Vertices`, a new run going on from it), with a group or body placed off the origin
  and turned.
- `Velocities`: initial velocities of a soft and a flexible body move them (the centre
  of mass moving at that velocity in free fall without gravity), a single row expanding,
  `CurrentVelocities` after the run.
- Particle groups: a filled shape moved and turned without filling again, filled again
  when `Spacing`/`Radius` changes; `add`/`remove` in world coordinates.
- Aliases: `Positions` and `Damping` warn and act as the new names.
- Gateway: contract tests of the soft body `velocities` and the flexible initial
  velocities.

## Open questions

1. The alias `Positions` reads the world points: keep it read/write (it was), or make it
   read-only like `CurrentVertices`.
2. Writing `CurrentVertices`/`CurrentVelocities` of a mesh during the run (teleport,
   kick): not now; Jolt could, the flexible solver would need its state reset.

## Phases

Each a step of its own, reviewed before the next:

1. Particle groups: `Vertices` in the group frame, `CurrentVertices`, the alias
   `Positions`, filling again only on `Spacing`/`Radius`, `add`/`remove`; tests.
   (Done: `Position`/`Orientation` of a group in a running simulation are an error,
   `phx:<Class>:running`; the constructor error for a Shape with points is
   `phx:<Class>:shapeAndVertices` (was `shapeAndPositions`); the alias `Positions`
   reads and writes world points, warning `phx:<Class>:renamedProperty`; the three
   demos with `Positions` moved at once, so that none warns.)
2. Meshes: `CurrentVertices` after the run (kept in `stateTransfer`),
   `CurrentVertices`/`CurrentVelocities` on all four; tests. (Done: the last vertices
   kept until the rest shape changes (`Vertices`, `Faces`/`Tetrahedra`, `Shape`,
   `Position`, `Orientation`); `CurrentVelocities` of the particle groups. That of the
   meshes needs the engine's soft body velocities: phase 4. The last state is not saved
   with the body (decided after the review, 2026-11-21): loading runs the setters of the
   rest shape, which would clear it anyway, and a loaded body starts from its rest shape
   as a new simulation does.)
3. Engine: soft body `velocities`, the initial velocities of the flexible solver
   (tools/jolt, JOLT_CHANGES.md); contract tests; MEX built. (Done: the soft body field
   `velocities`, protocol 2 step Z, JOLT_CHANGES.md item 45, Jolt `94b834c`: setting
   velocities restarts BDF2 with a backward Euler step; a pinned vertex keeps zero.)
4. `Velocities` and `CurrentVelocities` of soft and flexible bodies; tests. (Done:
   `Velocities` read when the body is created, like the rest shape; kept as set, so
   that one row follows a new mesh, while rows that no longer fit fail
   (`phx:<Class>:sizeMismatch`) when they are set or used; the vertices of the edges of
   a Kx4 mesh get the mean of their ends. The last velocities kept after a run as the
   last vertices are. A single row expands on the particle groups too.)
5. `LinearDamping` of soft and flexible bodies with the `Damping` alias; tests. (Done:
   `phxex_cantilever`, `phxex_plow` and the probe `flexibleFactor` moved. A scene saved
   by 2.0 with such a body warns once when loaded: MATLAB sets the saved `Damping`
   through the alias; saving it again ends that.)
6. Demos, help, user guide, skills, upgrade guide. (Done in three steps: the reference
   pages and class help, `phx.Particles.html` brought up to the group frame and `Shape`
   as well; the user guide and the product page's conventions; the skill
   `phx-scene-basics`. The demos had moved in phases 1 and 5. The upgrade guide from 1.x
   needs nothing: 1.x had no soft bodies or particles; the renamed 2.0 names belong in
   the release notes.)
