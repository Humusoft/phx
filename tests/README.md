# PHX test suite

Class-based `matlab.unittest` tests for the PHX API.

## Running

```matlab
cd tests
runtests_phx            % everything
runtests_phx("noengine")% skip the MEX integration tests
runtests_phx("pure")    % headless: no engine, no graphics
```

`runtests_phx` adds `../phx` to the path for the duration of the run.

## Shared helpers

Every test class derives from `PhxTestCase`, which holds what they all need:
`requireEngine` (assume the MEX away when it is missing), `spawnBody` (a
headless `phx.Body`), `spawnBodyOn` (a body drawn into given axes),
`prepareAxes` (an axes in an invisible figure), `bodyShape`,
`verifyAnchorsCoincide`, the teardown `PhxTestCase.deleteIfValid` (for objects
a test may delete itself) and the temporary-file writers `writeTextFile` /
`writeSTL`. It declares no tests of its own, so it adds nothing to the suite.

Prefer headless bodies (`[]` axes) unless the test is about the drawing.

## Layers (by test tag)

| File | Needs | Tag | What it locks in |
|------|-------|-----|------------------|
| `tPhxMath` | nothing | — | rotation matrices, point transform, decomposition round-trips, `alignZ` |
| `tShapeMass` | nothing | — | analytic mass & inertia of the solids; `computeVolume` (volume, moments per unit density, centre) of every shape, `computeMass` as those at the shape's `Density`, the `Terrain` placeholders |
| `tShapeMesh` | nothing | — | meshes the shapes generate for flexible and soft bodies: the divisions `ElementResolution` gives, a volumetric mesh that fills the shape exactly with matching faces, a closed outward surface mesh without inner vertices, the geodesic surface and the curved quadratic volume of a sphere, the curved volume of a cylinder and of a truncated or sharp cone along their axes and their divisions, the evenly spaced ring surface of a cone, the volume of a capsule joined from a cylinder and two half balls and its ring surface, the polar volume and ring surface of a straight, tapered or end-closed tube wall, the errors of shapes that generate none |
| `tBodyKinematics` | graphics | `Graphics` | pose property round-trips, input validation |
| `tBodyMass` | nothing (one case engine) | — / `Engine` | `Density`, `Volume`, `Mass` and `Inertia` of `phx.Body`: the shape's geometry at the density, mass and density scaling the inertia, the inertia changing neither, a new shape keeping the density, the fixed constructor order in every argument order, density and mass refused together, a PHX 2.0 shape density warning and taken, positive values, a flat mesh with a positive volume, a mass written during the run reaching the engine |
| `tInternals` | nothing (one `Graphics` case) | — / `Graphics` | the undocumented MATLAB internals PHX deliberately uses (perf) still exist and behave |
| `tReadPly` | nothing | — | the pure-MATLAB PLY reader: ASCII/binary (both endiannesses), vertices, normals, color, polygon fan-triangulation |
| `tExtrusion` | nothing | — | the rotation-minimizing frame of the swept-profile generator |
| `tTextureMapping` | nothing (some `Graphics` cases) | — / `Graphics` | texture orientation: image files and texture matrices laid out with u along the columns, `uint8` gray levels kept and other data stretched, to gray or through `TextureColormap`, built-in names drawing their generator output (and `"metal"` warning as the old name of `"scratches"`), the generated shapes unmirrored from outside with the image top at the profile/spine end, Terrain following its Height matrix, OBJ/PLY coordinates flipped into the image's v |
| `tSimulation` | engine (one `Graphics` case) | `Engine` / `Graphics` | free fall, static bodies, time accumulation, determinism, draw-path independence of mesh collision hulls; the contact properties of `Body` reaching the engine (rolling resistance and spin radius, contact stiffness sinking a box m g / k, contact damping setting the rebound) |
| `tPipelineRebuild` | engine (two `Graphics` cases) | `Engine` / `Graphics` | rebuilding the execution pipelines: a body added or deleted mid-run, body identity across a rebuild, invisible bodies kept out of the redraw, and the mid-run `delete` that used to crash the process |
| `tStoredState` | graphics (transfer tests also engine) | `Graphics` / `Engine` | the named kinematic-state store (`storeState`/`restoreState`/`clearStates`) and the pose & velocity transfer across engine initialization and teardown |
| `tEngineProtocol` | engine | `Engine` | contract of `phxEngine` itself, without `phx.*` objects: the verbs, the shapes of their inputs and outputs and the physics they set (shapes, the spherical, capsule and box shells holding and bearing bodies, meshes, terrains, shells and liquids, contacts, friction and restitution, joints with their limits changed at run time, motors and applied joint loads, persistent body loads, kinematic bodies); `MexFolder` tests another build of the engine |
| `tEngineSettings` | engine (convergence tests only) | — / `Engine` | `phx.engine.Settings`: the metre defaults, the tolerances following `LengthScale` in automatic mode, a value set switching its mode to manual and staying put, `"auto"` restoring the derived value, construction order, validation; the solver tolerances and iteration caps handed to the engine, islands ending at their tolerance or their own cap (counted by kind), `JointIterations` 0 falling back to `ContactIterations`, a looser tolerance taking fewer iterations, more `PositionIterations` closing a joint's drift; `phx.engine.BulletSettings` of PHX 1.x warning and returning `Settings` with the scale and margin |
| `tForceApplication` | engine | `Engine` | local/world force & torque frames, absolute vs COM-relative point of application, as one table of cases (guards the 2026-07-17 wrapper fixes); `AppliedForce`/`AppliedTorque` acting in every step until changed |
| `tSurfaceVelocity` | graphics (drive tests also engine) | `Graphics` / `Engine` | the conveyor-belt surface drive: the commanded speed reached, the vector read in the body frame, friction as its only path, the reaction on a belt free to move, and the property refusing a non-finite value |
| `tFieldForces` | nothing | — | the monopole and dipole force kernels (`phx.internal.computeMonopoleForces`/`computeDipoleForces`): Coulomb law, sign of `Attractivity`, 1/r², Newton's third law, own dipole poles not interacting, against an independent all-pairs sum |
| `tBuoyancy` | engine (floating tests only) | — / `Engine` | voxel volume vs analytic, sampling determinism, the per-step kernels (`phx.internal.computeBuoyancy`/`computeBuoyancyXY`: submerged volume, local center of buoyancy, flat and per-point level, bodies independent in one call, against a per-body reference), floating equilibrium, sinking |
| `tResistance` | nothing | — | the former names `EnvironmentVelocity`/`EnvironmentTwist` of `phx.Resistance`: read and write the new properties, with the warning `phx:Resistance:renamedProperty` |
| `tJointContract` | engine (property tests need nothing) | — / `Engine` | what every joint class shares: frame-property round-trips, the Z default axis, NaN feedback before initialization, holding two bodies together |
| `tGenericJoint` | engine | `Engine` | the meaning of the 6-DOF limits, one row per axis (free / locked / bounded), the right-handed sense of the angular ones, limits set during a run taking effect, rotated joint frames |
| `tCylindricalJoint` | engine | `Engine` | the 2 free DOF of the cylindrical joint (slide along and spin about the axis) against the 4 locked ones, the axis following a turned joint frame, and the roll/offset mismatches along the free DOF staying inert |
| `tJointFrames` | engine | `Engine` | the joint-frame coincidence rule documented on `phx.base.Joint`: a mismatch along a *free* DOF is inert, a mismatch in a *constrained* one is pulled out gradually, a `FixedJoint` leaves no free direction |
| `tJointMotors` | engine (two property cases need graphics only) | `Engine` / `Graphics` | the revolute and prismatic motor: reaching and holding `TargetVelocity`, saturating into a pure torque/force source, the zero-target brake against a load, a zero limit meaning off, both properties writing through to a running world, the motor effort appearing in the reaction feedback, and `AppliedTorque`/`AppliedForce` acting together with the motor |
| `tJointLimits` | engine (property cases need graphics only) | `Engine` / `Graphics` | the joint limits under one set of rules: `Limits` of the revolute and prismatic joint, `LinearLimits`/`AngularLimits` of the cylindrical one and, one row per axis, of the generic and bushing joint, `ConeAngle`/`TwistLimits` of the spherical one; free by default (rigid for the generic joint), a scalar locking (a circular cone), reversed/NaN/one-sided angular limits rejected in any row, stopping at either limit of a range without the zero, limits changed in a running world, a stop carrying the load in the reaction feedback, also against a bushing spring, the elliptic cone stopping a swing at the half-angle of its axis, a locked twist turning both shafts of a constant-velocity joint alike |
| `tRevolutionEnvelope` | engine | `Engine` | the bounding-cylinder collision envelope of `phx.shape.Revolution` |
| `tTube` | nothing (mesh cases graphics, envelope cases engine) | — / `Graphics` / `Engine` | the bore, wall and outer size of `phx.shape.Tube`, mass against the annulus formulas, the tapered and closed bore, a closed outward-facing mesh, and a body built for every envelope with the concave bore left open |
| `tSphereShell` | nothing (drawing cases graphics, body case engine) | — / `Graphics` / `Engine` | the outer `Diameter` and the wall of `phx.shape.SphereShell` growing inwards, the wall checked where the shape is used, mass and inertia against the shell formulas, the outer face and the inner face turned to the cavity, `Alpha` as the opacity of the whole wall (each face 1 - sqrt(1 - Alpha)), a ball held in a static shell and a dynamic one resting on a floor |
| `tCapsuleShell` | nothing (drawing cases graphics, body case engine) | — / `Graphics` / `Engine` | the outer `Diameter` and the wall of `phx.shape.CapsuleShell` growing inwards, the wall checked where the shape is used, mass and inertia of the solid capsule less its cavity along every `Axis` and the thin-wall limit, the outer face and the inner face turned to the cavity around the segment of the cylindrical part, `Alpha` as the opacity of the whole wall, a ball held in a static upright shell and a dynamic drum resting on a floor |
| `tBoxShell` | nothing (drawing cases graphics, body case engine) | — / `Graphics` / `Engine` | the outer `Size` and the wall of `phx.shape.BoxShell` growing inwards, the wall and `InnerEdgeRadius` checked where the shape is used, the mass of the rounded box less its cavity (sharp and rounded) against the closed form, both faces closed, at the rounding distances from the core and facing out of the wall, `Alpha` as the opacity of the whole wall, a ball held in a static shell and a dynamic crate resting on a floor |
| `tSpringElements` | engine (configuration tests need nothing) | — / `Engine` | `Spring`, `Rope` and `TorsionSpring`: geometry, energy and errors before a simulation; a hanging mass at the extension m*g/k, oscillation periods with and without inertance, a stiffness change reaching a running simulation, rope tension over a pulley, a torsion spring winding past half a turn |
| `tSoftBodies` | engine (configuration tests need nothing) | — / `Engine` | technical contract only (their physics is still tuned): the errors, rest shape, surface, mass and quadratic edge vertices of `SoftBody` and `FlexibleBody`, a `FlexibleBody` meshed from its `Shape` (its own `Density` or `Mass`, one of them, 1000 kg/m^3 by default; its `Volume`; a PHX 2.0 shape density warning and taken unless the body has its own) and a `SoftBody` covered by the surface of its `Shape` (mass its own, a shape density warning and ignored); a simulation moves them, and `IntersectionWeld` holds the vertices its body overlaps and goes with its body |
| `tParticles` | engine (bookkeeping tests need nothing) | — / `Engine` | `Particles` bookkeeping (add, remove, sizes, limits, bounding box, a `Shape` filled with touching particles) and, running, reads and writes through the engine, a force giving its particle F*dt/m, rest on a floor, state kept after the simulation |
| `tLiquid` | engine (bookkeeping tests need nothing) | — / `Engine` | `Liquid` defaults, the mass of its particles, no particle limit, a `Shape` filled on the lattice of the `Spacing` (six shapes within 3 % of their volume, a box exactly, the pose, the order of the constructor's arguments, filled again until `Positions` replace it), the errors; running, the engine taking that mass, a lattice reading the `Density` (also changed at run time), a column keeping its level in a tank, the shape done once simulated, state kept after the simulation; assumed away on an engine without liquids |
| `tZone` | graphics (pipeline tests also engine) | `Graphics` / `Engine` | entry/exit detection, the static-anchor watch rule, passive zones, seeding on rebuild |
| `tRaycast` | graphics (geometry tests also engine) | `Graphics` / `Engine` | hit point/normal/distance, anchor-local rays, NaN misses, both sensor modes, the objectID to phx.Body lookup |
| `tPlanarShadow` | graphics (projection tests also engine) | `Graphics` / `Engine` | shadow projection onto the plane, light models, anchoring, extent, decimation |
| `tAssemblyConventions` | graphics | — / `Graphics` | the four rules every `phx.assembly` builder obeys: axes target, headless build, rigid base pose, conflicting rotation options |
| `tAssembly` | graphics (three `Engine` cases) | `Graphics` / `Engine` | what each builder lays out: arena dimensions, chain link poses and joint types, scatter placement and density, the running bond of a wall; plus a ball kept inside the arena, a swinging chain and a collapsing wall |
| `tImport` | graphics (two `Engine` cases) | — / `Graphics` / `Engine` | the URDF importer: file/XML errors, object structure, name sanitization, link world poses against an independent rpy implementation, mesh path resolution, joint substitution |
| `tPhxSimulinkRefs` | nothing | — | `ParameterReference` parsing and the `BlockBackend` port-reference helpers |
| `tPhxModelSource` | nothing (run cases engine and Simulink) | — / `Engine` | the scene source of the PHX Model block: a scene function by name or by path and a MAT file give the same scene, other files are rejected, and every kind runs in the block |
| `tSkillsPackaged` | nothing (source checkout only) | — | the AI agent skills in `phx/skills` (shipped in the `.mltbx`) stay identical to the working ones in `.claude/skills`, front-matter names match their folders, and `package.ignore` does not exclude them |
| `tExampleGallery` | nothing (source checkout only) | — | the generated Examples gallery, its thumbnails and the toc stay in sync with `examples/` |
| `tDocLinks` | nothing (source checkout only) | — | no HTML help page links to or shows a file that is not there; Contents.m, the reference toc and the product page group the classes alike |

- **`Graphics`** — the test creates figures (invisible ones), so a display-capable session is needed. Headless bodies (`[]` axes) need no display and run untagged.
- **`Engine`** — needs the `phxEngine` MEX; gracefully *assumed away* (filtered, not failed) when absent.

## Next steps (not yet covered)

- The springs and ropes (constraint feedback, spring force sign) - deferred until it is
  decided whether the spring calculation moves into the engine, so the tests can be
  written against the final implementation.
- Per-shape collision envelopes beyond the one of `phx.shape.Revolution`.
