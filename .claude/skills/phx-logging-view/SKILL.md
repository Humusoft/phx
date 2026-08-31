---
name: phx-logging-view
description: >
  Record and observe a PHX simulation — log object properties over time with
  phx.Logger, draw motion trails with phx.Trace, measure distances/velocities
  between bodies with phx.Measure, detect and count bodies in a region with
  phx.Zone, cast shadows onto a plane with phx.PlanarShadow (only when the user
  asks for them), and set up the view with phx.Camera and the interactive
  phx.extra.Viewer. Use when capturing signals for plots, detecting events in a
  scene, or configuring how a PHX scene is displayed.
---

# PHX logging & visualization

## phx.Logger — record properties over time

A logger records the value of one or more properties of one or more objects each
step (down-sampled to a target `Frequency`). All listed objects must share the
named properties.

```matlab
% One property of one body
L = phx.Logger(boxB, "Parameters", "LinearVelocity", "Frequency", 100);

% Multiple properties of one object
L2 = phx.Logger(joint, "Parameters", ["ForceA", "ForceB"], "Frequency", 100);

% One property across several objects (e.g. a body's child springs)
L3 = phx.Logger(box.Children([1 3]), "Parameters", "Force", "Frequency", 10);
```

Read the results after the run:

```matlab
L.Time            % time axis (column vector; may be irregular if Frequency
                  % isn't divisible by the step rate)
L.Data            % matrix of all channels concatenated
L.getChannel(1)   % data of a single channel (n-by-width)
L.dispChannels    % print the channel layout

plot(L.Time, L.Data);
mag = @(x) sqrt(sum(x.^2, 2));            % magnitude of a vector channel
plot(L.Time, mag(L.getChannel(1)));
```

`Frequency` is a *target*; the achievable rate is capped by the simulation step
size. Anything readable as a property (pose, velocity, force, torque, energy,
joint angle, spring force/length, thruster thrust, …) can be logged.

## phx.Trace — motion trail

Draws the recent path of a point on a body:

```matlab
phx.Trace(body, "TracePoints", 1200, "Overlay", true, "Color", [1 1 0]);
phx.Trace(box,  "TracePoints", 50, "Color", [0.5 0.5 0.5]);
```

`TracePoints` is the trail length (number of retained points); `Overlay` draws it
on top of geometry.

## phx.Zone — spatial detection zone

A box-shaped region anchored to a body (so it moves with a moving anchor) that
reports which bodies are inside. Use it for counting, sorting, catch basins,
end-of-conveyor detection, "did it reach the target area" checks.

```matlab
% Active zone: evaluated every step, callbacks fire on the transitions
zone = phx.Zone(ground, "Bodies", rocks, ...       % watch set (default: all
    "Position", [-30 0 5], ...                     %   non-static bodies)
    "Size", [40 50 10], ...                        % full box dims, anchor-local
    "EulerAngles", [0 0 0], ...
    "EnteredFcn", @(z, b) disp(z.Count), "ExitedFcn", []);

zone.Contents        % phx.Body array currently inside
zone.Count           % how many
zone.EnteredCount    % cumulative throughput since the last pipeline rebuild
zone.ExitedCount
```

**Passive zones cost nothing per substep.** Create the zone with
`"SimulationOrder", "none"` and call `update(zones)` when you actually want the
answer — it recomputes occupancy on the spot (and fires the callbacks). This is the
right shape for tallies read once at the end, and it takes an array:

```matlab
bins = phx.Zone.empty;
for k = 0:nRows
    bins(k+1) = phx.Zone(floor, "Position", [...], "Size", [...], ...
        "SimulationOrder", "none", "Visible", false);
end
% ... run the simulation ...
update(bins);                       % one detection tick for the whole array
counts = [bins.Count]';
set(bins(3).Contents, "Color", [1 0 0]);
```

Notes: `Visible`/`Overlay` control the drawn translucent box; the watch set is
resolved at pipeline build time (a scene change re-enumerates it), and a zone
anchored to a **static** body watches only non-static bodies.
Demos: `phxex_galton` (passive bins), `phxex_soil` (active zone + HUD).

## phx.Measure — live measurement

`phx.Measure(bodyA, bodyB)` (optionally `phx.Measure(bodyA, bodyB, pointA, pointB)`,
or via `PointA`/`PointB`) reports mutual kinematics between two bodies live during the
simulation and draws the link into the scene. Dependent readouts, all loggable:
`Distance` (scalar), `Position` (vector between the points, global), `PositionInA` /
`PositionInB` (in the local frame of the respective body), and the matching
`Velocity` / `VelocityInA` / `VelocityInB`. `Overlay` draws it on top of geometry.

## phx.Camera — scriptable camera

`phx.Camera` is a simulable object, so a camera pose can be scripted with
`phx.Script` or follow a body, useful for fly-throughs and recorded videos.

## phx.PlanarShadow — shadow on a plane (on request only)

**Never add a shadow on your own initiative.** It costs per-frame work proportional to
the mesh size of the cast bodies, and the result is a projection rather than a rendered
shadow, so it comes with the visual limits listed below. Add it only when the user asks
for a shadow, or asks for something a shadow is the answer to (for example "it is hard
to tell how high the body is" or "make the scene look better for a video").

When asked: `phx.PlanarShadow(bodies)` projects the visual geometry of the given bodies
onto a plane and draws it as a flat translucent silhouette. It is a graphics object
only, with no effect on the physics.

```matlab
phx.PlanarShadow([b1 b2], "LightDirection", [-0.4 -0.3 -1]);   % ground plane z = 0

% plane carried by a body: Position/Normal are in the anchor's local frame,
% so the shadow tilts and travels with the plate
phx.PlanarShadow(ball, "Anchor", plate, "Position", [0 0 thickness/2], ...
    "Extent", [2 2], "Alpha", 0.35);
```

Plane: `Position` + `Normal` (in the `Anchor` frame when one is set, world otherwise).
Light: `LightDirection` (parallel, default straight down) or `LightPosition` (point
source). Appearance: `Color`, `Alpha`, `Offset` (lift above the surface), `Extent`
(half-sizes that keep the shadow on a finite plate). `Detail` decimates the cached
geometry: a few percent on ordinary shapes, several times cheaper on a body with a very
large mesh (tens of thousands of vertices), which is where it is worth reaching for.

Limits worth knowing, and worth telling the user about when they ask for shadows: one
plane and one light per object (create several for more), bodies neither shadow each
other nor block a shadow, `Extent` clamps rather than clips, a body is dropped once its
centre passes behind the plane, and assigning `Color` resets `Alpha` (set `Alpha`
afterwards). Cost follows the total mesh size of the cast bodies, not their number: a
pile of simple bodies is cheap, one big imported mesh is not.

## phx.extra.Viewer — the default way to show a scene

`phx.extra.Viewer` is an enhanced figure/axes for interactive exploration (orbit,
pan, zoom, object selection/drag, free-run, sky textures). **This is the default
container for a PHX scene** — reach for plain axes only for throwaway/minimal
scenes, subplot or app layouts, and headless `[]` runs (see phx-scene-basics).
Open it with the `"clear"` verb and capture the axes it returns:

```matlab
figure(1);
[viewer, ax] = phx.extra.Viewer("clear", ...
    "DefaultCameraPosition", [-12 -8 5], ...
    "DefaultCameraTarget",  [2 0 3], ...
    "ViewMode", "plain", ...               % "texture" | "axis" | "plain"
    "Lighting", "studio", ...              % "headlight" | "studio" | "none"
    "Texture", "nebula");                  % sky texture (see below)

% ... build the scene into ax, e.g. phx.Body(ax, ...) ...

viewer.displayText("Lift-off...");         % HUD text overlay
sim = phx.Simulation(ax);
```

Pass the returned `ax` as the first argument to every `phx.Body(ax, ...)` (and to the
`phx.assembly.*` builders) so the scene lands in the viewer's axes. Mouse: left drag
pans, middle drag orbits, scroll zooms, double click selects. Keys: F1 help, F2
lighting mode, F3 view mode, F5 free-run, Home default view, PgUp/PgDn cycle views.

**`Texture`.** The equirectangular background of the sky sphere (shown in `ViewMode`
`"texture"`). Takes an image path *or* the name of a built-in texture bundled with the
toolbox — **`"sky"` (the default), `"nebula"`, `"gradient"`, `"checker"`, `"tiles"`** —
matched case-insensitively. An unknown name errors (`phx:Viewer:fileNotFound`) rather
than falling back, so do not invent names.

> The viewer is also what makes **shape** textures visible at all: only its axes take the
> world-primitive draw path that carries texture data. In plain axes a textured
> `phx.shape.*` silently renders untextured (just its `Color`) — nothing errors. If the
> scene uses `"Texture"` on any shape, build it into the viewer's `ax`.

**`Lighting`.** `"headlight"` (the default) puts one light at the camera, so whatever you
look at is lit — safe but flat, since the shading then carries no directional information.
`"studio"` uses three world-fixed lights that stay put while you orbit, so a body's shading
conveys its shape and its silhouette separates from the background; prefer it for
screenshots, videos and anything where form matters. `"none"` leaves the lighting to you.
The modes are deliberately exclusive: a headlight shines along the view axis, so mixing it
in would add a view-locked gradient that fights the studio key's directional one.

The viewer takes over the **whole figure** — it is not designed to share a window with
other plots, so put result plots in a separate figure. It runs fine under `-batch`
(invisibly), so headless verification is no reason to drop it.

## Plain MATLAB axes (the exception)

For minimal scenes and for visualizations that must live in a subplot or app layout,
configure the axes directly and let `phx.Body`/`phx.Simulation` default to `gca` (or
pass your own `ax`, as `phxex_multiview` does with two subplots):

```matlab
clf; view(3); axis("equal"); grid("on"); camlight("headlight");
axis("manual");   % freeze limits so falling bodies don't rescale the view
```

For a run with no visualization at all, pass `[]` as the axes target
(`phx.Body([], ...)`) — that skips the graphics work entirely, unlike merely running
headless.

After the run, plot logged data into a second figure (`figure(2); plot(...)`), as
the demos do — keep the simulation axes and the result plots separate.

## Related skills

- **phx-scene-basics** — bodies, shapes, `Simulation.step`, running headless.
- **phx-constraints-forces** — the elements whose `Force`/`Angle`/`Thrust` you typically
  log, plus `phx.Function` for per-step logic reacting to what you observe.
- **phx-engine-gotchas** — engine quirks, error IDs, tests.
- **phx-simulink** — exchange signals with a PHX scene from Simulink via the PhxModel block.
