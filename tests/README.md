# PHX test suite

Class-based `matlab.unittest` tests for the Handle Physics API.

## Running

```matlab
cd tests
runtests_phx            % everything
runtests_phx("noengine")% skip the MEX integration tests
runtests_phx("pure")    % headless: no engine, no graphics
```

`runtests_phx` adds `../phx` to the path for the duration of the run.

## Layers (by test tag)

| File | Needs | Tag | What it locks in |
|------|-------|-----|------------------|
| `tPhxMath` | nothing | — | rotation matrices, point transform, decomposition round-trips |
| `tShapeMass` | nothing | — | analytic mass & inertia of Box/Sphere/Cylinder |
| `tBodyKinematics` | graphics | `Graphics` | pose property round-trips, input validation |
| `tInternals` | nothing (one `Graphics` case) | — / `Graphics` | the undocumented MATLAB internals PHX deliberately uses (perf) still exist and behave |
| `tReadPly` | nothing | — | the pure-MATLAB PLY reader: ASCII/binary (both endiannesses), vertices, normals, color, polygon fan-triangulation |
| `tSimulation` | engine + graphics | `Engine` | free fall, static bodies, time accumulation, determinism |
| `tForceApplication` | engine + graphics | `Engine` | local/world force & torque frames, absolute vs COM-relative point of application (guards the 2026-07-17 wrapper fixes) |
| `tBuoyancy` | engine + graphics (floating tests only) | — / `Engine` | voxel volume vs analytic, sampling determinism, floating equilibrium, sinking |
| `tGenericJoint` | engine (frame helpers need nothing) | — / `Engine` | the meaning of the 6-DOF limits (free / locked / bounded), the right-handed sense of the angular ones, rotated joint frames, reaction feedback, `PointA`/`EulerAnglesA` round-trips |
| `tCylindricalJoint` | engine (feedback test needs nothing) | — / `Engine` | the 2 free DOF of the cylindrical joint (slide along and spin about the axis) against the 4 locked ones, the axis following a turned joint frame, and the roll/offset mismatches along the free DOF staying inert |
| `tSkillsPackaged` | nothing (source checkout only) | — | the AI agent skills in `phx/skills` (shipped in the `.mltbx`) stay identical to the working ones in `.claude/skills`, front-matter names match their folders, and `toolbox.ignore` does not exclude them |
| `tJointFrames` | engine (no graphics, `[]` axes) | `Engine` | the joint-frame coincidence rule documented on `phx.base.Joint`: a mismatch along a *free* DOF is inert (prismatic `PointA` along the sliding axis), a mismatch in a *constrained* one is pulled out gradually, a `FixedJoint` leaves no free direction |

- **`Graphics`** — a body owns an `hgtransform`, so a display-capable session is needed (invisible figures are used).
- **`Engine`** — needs the `phx.engine.io` MEX; gracefully *assumed away* (filtered, not failed) when absent.

## Next steps (not yet covered)

- The remaining joints and the springs (constraint feedback, spring force sign), per-shape collision envelopes.
- `addObjects`/`delete` rebuilding the execution pipelines correctly.
- `storeState`/`restoreState` velocity transfer across re-initialization.
