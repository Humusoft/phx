# PHX test suite

Class-based `matlab.unittest` tests for the Handle Physics API.

## Running

```matlab
cd tests
runtests_phx            % everything
runtests_phx("noengine")% skip the MEX integration tests
runtests_phx("pure")    % headless: no engine, no graphics, no add-on toolboxes
```

`runtests_phx` adds `../phx` to the path for the duration of the run.

## Layers (by test tag)

| File | Needs | Tag | What it locks in |
|------|-------|-----|------------------|
| `tPhxMath` | nothing | — / `Toolbox` | rotation matrices, point transform, decomposition round-trips |
| `tShapeMass` | nothing | — | analytic mass & inertia of Box/Sphere/Cylinder |
| `tBodyKinematics` | graphics | `Graphics`, `Toolbox` | pose property round-trips, input validation |
| `tSimulation` | engine + graphics | `Engine` | free fall, static bodies, time accumulation, determinism |
| `tBuoyancy` | engine + graphics (floating tests only) | — / `Engine` | voxel volume vs analytic, sampling determinism, floating equilibrium, sinking |

- **`Toolbox`** — relies on `robotics.internal.*` (Robotics System / Navigation Toolbox).
- **`Graphics`** — a body owns an `hgtransform`, so a display-capable session is needed (invisible figures are used).
- **`Engine`** — needs the `phx.engine.io` MEX; gracefully *assumed away* (filtered, not failed) when absent.

## Next steps (not yet covered)

- Joints/springs (constraint feedback, spring force sign), per-shape collision envelopes.
- `addObjects`/`delete` rebuilding the execution pipelines correctly.
- `storeState`/`restoreState` velocity transfer across re-initialization.
