# Mass properties: proposal (2026-11-21)

Status: agreed in discussion, not implemented. Covers `phx.Body`, the `phx.shape.*`
classes, `phx.FlexibleBody`, `phx.SoftBody`, `phx.Particles`, `phx.Liquid` and the
`phx.assembly` builders.

## Why

1. **The density sits in the shape.** Every `phx.shape.*` has a `Density` and
   `computeMass` multiplies by it. A shape is geometry and appearance; the material is
   the owner's. Today `phx.Body` and `phx.FlexibleBody` take the shape's density,
   `phx.SoftBody`, `phx.Particles` and `phx.Liquid` silently ignore it (they have
   their own `Mass` or `Density`).
2. **`Mass` does not update `Inertia`.** `phx.Body` copies mass, inertia and centre of
   mass from the shape once. Changing `Mass` afterwards leaves the inertia of the old
   mass, so the body turns too easily or too hard.
3. **The order of constructor arguments matters.** `applyArguments` sets the properties
   in the order given, so `phx.Body("Mass", 5, "Shape", ...)` loses the mass to the
   shape, while `phx.Body("Shape", ..., "Mass", 5)` keeps it.
4. **Zero means something else.** The engine takes a zero mass as a frozen body and a
   zero inertia component as a rotation locked about that axis, the opposite of what a
   zero moment means physically. No demo and no PHX help uses it; only the gateway
   protocol describes it.

## Principles

- A shape gives geometry: volume, centre of volume, and the inertia of its volume at
  unit density. Its owner gives the material.
- One scaling rule: the mass scales the inertia, keeping the distribution of mass; the
  inertia never changes the mass.
- No modes and no hidden state: every result follows from the visible properties and
  a fixed order.
- Mass, density and moments of inertia are positive.

## Shapes

- New method, replacing `computeMass`:

  ```matlab
  [volume, inertia, center] = shape.computeVolume
  ```

  `volume` in m^3, `center` the centre of volume in the shape frame (m), `inertia` the
  principal moments about it, in the shape axes, of the volume at unit density
  (kg*m^2 per kg/m^3, that is m^5). The body's mass properties are those times its
  density.
- Shapes keep their special cases as geometry: an open STL weighed as its bounding box,
  `Rock` as its sphere, `SphereShell`/`CapsuleShell`/`BoxShell` as their walls.
  `Terrain` has no meaningful volume and gives a documented placeholder (positive
  volume and moments, as its 1000 kg and [100 100 100] today), for the unusual dynamic
  terrain.
- `Density` of a shape becomes deprecated (see Compatibility).

## phx.Body

| Property | Access | Meaning |
|---|---|---|
| `Density` | read/write, > 0, default 1000 | kg/m^3, `Mass/Volume` |
| `Volume` | read-only | m^3, from the shape |
| `Mass` | read/write, > 0 | kg, `Density*Volume` |
| `Inertia` | read/write, 1x3, each > 0 | kg*m^2, principal moments about the centre of mass in the body axes |
| `CenterOfMass` | read/write | m, in the body frame (as today) |

What a write changes:

| Write | Density | Mass | Inertia | CenterOfMass |
|---|---|---|---|---|
| `Density` | set | `Density*Volume` | scaled by the mass ratio | - |
| `Mass` | `Mass/Volume` | set | scaled by the mass ratio | - |
| `Inertia` | - | - | set | - |
| `CenterOfMass` | - | - | - | set |
| `Shape` | kept | `Density*Volume` of the new shape | from the new shape | from the new shape |

- Scaling by the mass ratio keeps the distribution of mass, the radii of gyration
  sqrt(I/m): a custom `Inertia` (a hollow rotor, mass near an axis) survives later
  changes of mass or density. A new shape replaces it with that of the shape.
- No property holds the distribution: with positive mass it is `Inertia/Mass`.
- **Constructor order**, whatever the order of the arguments: `Shape`, then `Density`
  or `Mass` (both given is an error), then `Inertia`, then `CenterOfMass`. So
  `phx.Body("Shape", s, "Mass", 5, "Inertia", I)` gives exactly 5 kg and `I`.
- A body always has a shape (the constructor draws the default box), so it always has
  a volume and a mass.
- While the simulation runs, every write reaches the engine (`massprops`), as today.
  For a static or kinematic body the values are kept and play no part.
- **Locked rotation** is no longer a zero inertia. If it is needed, it comes as its own
  explicit property (open question below); a joint to a static body
  (`phx.GenericJoint`) does it meanwhile. The engine protocol keeps its zero rules; PHX
  just never sends a zero.

## Other owners of shapes

| Class | Material | Change |
|---|---|---|
| `phx.FlexibleBody` | `Density` (read/write), `Mass` (read/write, `Density*Volume`) | add `Volume` (read-only, of the tetrahedra); the shape gives only the mesh, its `Density` no longer |
| `phx.SoftBody` | `Mass` (read/write, total, spread over the vertices) | none; a surface has no volume of material, the shape gives only the mesh |
| `phx.Particles` | `Density`, `ParticleMass`, `Mass` | none; the shape gives only the region to fill |
| `phx.Liquid` | `Density` (the liquid at rest), `ParticleMass` and `Mass` read-only | none; the shape gives only the region to fill |

Inertia does not apply to the deformable bodies and particle groups: it follows from
their vertices or spheres.

## phx.assembly

- `chain` and `wall`: their `Density` option goes to the bodies' `Density`, not into
  the shapes.
- `import`: the `Density` option of meshes goes to the bodies; a URDF's `<inertial>`
  sets `Mass`, `Inertia` and `CenterOfMass` in the constructor order above.

## Compatibility (2.0.0 has shape densities)

- Scripts: a shape still accepts `Density`, with a warning `phx:Shape:densityMoved`
  ("give the density to the body"). A body, flexible body, soft body or particle group
  receiving such a shape takes the shape's density only if it was not given its own
  `Density` or `Mass` (soft bodies and particle groups: the warning says they use
  their own `Mass`/`Density`). The deprecated property stays hidden for some releases,
  then goes.
- Saved scenes (`saved_*.mat`, `Simulation` MAT files): the shapes saved inside the
  graphics keep their `Density` property, so they load. A body stores `Mass`,
  `Inertia`, `CenterOfMass` and `Volume`, with `Density` dependent (`Mass/Volume`), so
  its saved mass and inertia load unchanged; a file from before has no `Volume`,
  which loading takes from the saved shape.
- Behaviour that changes: setting `Mass` (or `Density`) now scales the inertia; a zero
  mass or inertia component is an error. Both go into the upgrade guide.
- The 28 demos (58 shape densities), the assembly builders, the help, the user guide,
  the skills and the upgrade guide move to body densities.

## Implementation notes

- `phx.Body` stores `Mass`, `Inertia`, `CenterOfMass` and `Volume`; `Density` is
  dependent. Storing the mass keeps the engine's quantity, the saved files and the
  Simulink parameter references as they are.
- The fixed constructor order: the constructor takes `Shape`, `Density`, `Mass`,
  `Inertia` and `CenterOfMass` out of the name-value pairs and applies them in that
  order after the others.

## Tests

- Each shape: `computeVolume` against closed forms (box, sphere, cylinder, cone,
  capsule, shells, revolution, extrusion, mesh); the shell inertia as solid minus
  bore.
- Body: each row of the write table; the constructor order with the arguments in every
  order; a custom inertia surviving `Mass` and `Density` changes; a new shape
  replacing it; positive validators (zero and negative rejected); writes reaching a
  running simulation.
- Compatibility: a shape with `Density` warns and its density reaches a body without
  its own; a 2.0.0 `saved_*.mat` loads with its masses and inertias.
- FlexibleBody: `Volume` and `Mass = Density*Volume`; the shape's density ignored
  (with the warning).

## Open questions

1. Locked rotation: an explicit property, and its form (`Rotation` logical as in
   `phx.Particles`, or per axis, as Jolt's allowed degrees of freedom), or only
   joints.
2. The name of the shape method (`computeVolume`) and of the inertia per density it
   returns.
3. The `Terrain` placeholder values.
4. How long the shape `Density` stays deprecated before it goes.
5. Whether `phx.Particles` and `phx.Liquid` also get a `Volume`.

## Phases

Each a step of its own, reviewed before the next:

1. Shapes: `computeVolume`; `computeMass` in `phx.base.Shape` as those at the shape's
   `Density` (`Terrain`, without a density, at 1000 kg/m^3), so that everything works
   as before; tests. (Done: masses, moments and centres of all shapes as before to
   the last bits.)
2. `phx.Body`: the properties, the write rules, the constructor order, the positive
   validators; the shape `Density` moved to `phx.base.Shape`, hidden, NaN when not
   given, taken by the body when given; tests. (Done. A flat open mesh falls back to a
   box a thousandth of its largest side thick, so that its mass stays positive. A body
   saved before keeps its saved mass and inertia; its volume comes from its shape, or
   for a saved mesh, whose geometry only the graphics keep, from its mass and the
   density of its shape.)
3. `phx.FlexibleBody` (and the shape density of the others); tests. (Done:
   `FlexibleBody.Volume`; the constructor takes `Density` or `Mass` (both is an error)
   after the mesh, whatever the order of the arguments; a density given to the shape
   still sets the `Density`. `SoftBody`, `Particles` and `Liquid` never read the shape's
   density: nothing to change until the warning of phase 4.)
4. `phx.assembly` builders and the demos, with the warning `phx:Shape:densityMoved`
   for a density given to a shape (not before: until the demos move, it would warn in
   every one of them).
5. Saved scenes: loading, resaving the `saved_*.mat` files.
6. Help, user guide, skills, upgrade guide.
