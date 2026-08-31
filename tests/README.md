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
headless `phx.Body`), `prepareAxes` (an axes in an invisible figure),
`bodyShape`, `verifyAnchorsCoincide` and the temporary-file writers
`writeTextFile` / `writeSTL`. It declares no tests of its own, so it adds
nothing to the suite.

Prefer headless bodies (`[]` axes) unless the test is about the drawing.

## Layers (by test tag)

| File | Needs | Tag | What it locks in |
|------|-------|-----|------------------|
| `tPhxMath` | nothing | — | rotation matrices, point transform, decomposition round-trips, `alignZ` |
| `tShapeMass` | nothing | — | analytic mass & inertia of Box/Sphere/Cylinder |
| `tBodyKinematics` | graphics | `Graphics` | pose property round-trips, input validation |
| `tInternals` | nothing (one `Graphics` case) | — / `Graphics` | the undocumented MATLAB internals PHX deliberately uses (perf) still exist and behave |
| `tReadPly` | nothing | — | the pure-MATLAB PLY reader: ASCII/binary (both endiannesses), vertices, normals, color, polygon fan-triangulation |
| `tExtrusion` | nothing | — | the rotation-minimizing frame of the swept-profile generator |
| `tSimulation` | engine (one `Graphics` case) | `Engine` | free fall, static bodies, time accumulation, determinism, draw-path independence of mesh collision hulls |
| `tPipelineRebuild` | engine (two `Graphics` cases) | `Engine` / `Graphics` | rebuilding the execution pipelines: a body added or deleted mid-run, body identity across a rebuild, invisible bodies kept out of the redraw, and the mid-run `delete` that used to crash the process |
| `tStoredState` | graphics (transfer tests also engine) | `Graphics` / `Engine` | the named kinematic-state store (`storeState`/`restoreState`/`clearStates`) and the pose & velocity transfer across engine initialization and teardown |
| `tForceApplication` | engine | `Engine` | local/world force & torque frames, absolute vs COM-relative point of application, as one table of cases (guards the 2026-07-17 wrapper fixes) |
| `tBuoyancy` | engine (floating tests only) | — / `Engine` | voxel volume vs analytic, sampling determinism, floating equilibrium, sinking |
| `tJointContract` | engine (property tests need nothing) | — / `Engine` | what every joint class shares: frame-property round-trips, the Z default axis, NaN feedback before initialization, holding two bodies together |
| `tGenericJoint` | engine | `Engine` | the meaning of the 6-DOF limits (free / locked / bounded), the right-handed sense of the angular ones, rotated joint frames |
| `tCylindricalJoint` | engine | `Engine` | the 2 free DOF of the cylindrical joint (slide along and spin about the axis) against the 4 locked ones, the axis following a turned joint frame, and the roll/offset mismatches along the free DOF staying inert |
| `tJointFrames` | engine | `Engine` | the joint-frame coincidence rule documented on `phx.base.Joint`: a mismatch along a *free* DOF is inert, a mismatch in a *constrained* one is pulled out gradually, a `FixedJoint` leaves no free direction |
| `tRevolutionEnvelope` | engine | `Engine` | the bounding-cylinder collision envelope of `phx.shape.Revolution` |
| `tZone` | graphics (pipeline tests also engine) | `Graphics` / `Engine` | entry/exit detection, the static-anchor watch rule, passive zones, seeding on rebuild |
| `tPlanarShadow` | graphics (projection tests also engine) | `Graphics` / `Engine` | shadow projection onto the plane, light models, anchoring, extent, decimation |
| `tAssemblyConventions` | graphics | — / `Graphics` | the four rules every `phx.assembly` builder obeys: axes target, headless build, rigid base pose, conflicting rotation options |
| `tAssembly` | graphics (three `Engine` cases) | `Graphics` / `Engine` | what each builder lays out: arena dimensions, chain link poses and joint types, scatter placement, the running bond of a wall; plus a ball kept inside the arena, a swinging chain and a collapsing wall |
| `tImport` | graphics (two `Engine` cases) | — / `Graphics` / `Engine` | the URDF importer: file/XML errors, object structure, name sanitization, link world poses against an independent rpy implementation, mesh path resolution, joint substitution |
| `tPhxSimulinkRefs` | nothing | — | `ParameterReference` parsing and the `BlockBackend` port-reference helpers |
| `tSkillsPackaged` | nothing (source checkout only) | — | the AI agent skills in `phx/skills` (shipped in the `.mltbx`) stay identical to the working ones in `.claude/skills`, front-matter names match their folders, and `toolbox.ignore` does not exclude them |
| `tExampleGallery` | nothing (source checkout only) | — | the generated Examples gallery, its thumbnails and the toc stay in sync with `examples/` |
| `tDocLinks` | nothing (source checkout only) | — | no HTML help page links to or shows a file that is not there |

- **`Graphics`** — a body owns an `hgtransform`, so a display-capable session is needed (invisible figures are used).
- **`Engine`** — needs the `phx.engine.io` MEX; gracefully *assumed away* (filtered, not failed) when absent.

## Next steps (not yet covered)

- The springs and ropes (constraint feedback, spring force sign) - deferred until it is
  decided whether the spring calculation moves into the engine, so the tests can be
  written against the final implementation.
- Per-shape collision envelopes beyond the one of `phx.shape.Revolution`.
