---
name: phx-logging-view
description: >
  Record and visualize a PHX simulation — log object properties over time with
  phx.Logger, draw motion trails with phx.Trace, measure distances/angles with
  phx.Measure, and set up the view with phx.Camera and the interactive
  phx.extra.Viewer. Use when capturing signals for plots or configuring how a
  PHX scene is displayed.
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

## phx.Measure — live measurement

`phx.Measure` reports geometric quantities (distances/angles between points or
bodies) live during the simulation, drawn into the scene.

## phx.Camera — scriptable camera

`phx.Camera` is a simulable object, so a camera pose can be scripted with
`phx.Script` or follow a body, useful for fly-throughs and recorded videos.

## phx.extra.Viewer — interactive viewer

`phx.extra.Viewer` is an enhanced figure/axes for interactive exploration (orbit,
pan, zoom, object selection/drag, free-run, sky textures). Most demos open it with
the `"clear"` verb and capture the axes it returns:

```matlab
figure(1);
[viewer, ax] = phx.extra.Viewer("clear", ...
    "DefaultCameraPosition", [-12 -8 5], ...
    "DefaultCameraTarget",  [2 0 3], ...
    "ViewMode", "plain", ...               % "texture" | "axis" | "plain"
    "Texture", "defaultNebulaFFT");        % sky texture (res/ has more)

% ... build the scene into ax, e.g. phx.Body(ax, ...) ...

viewer.displayText("Lift-off...");         % HUD text overlay
sim = phx.Simulation(ax);
```

Pass the returned `ax` as the first argument to every `phx.Body(ax, ...)` so the
scene lands in the viewer's axes. Interactive keys: F1 help, F2 headlight, F3 view
mode, F5 free-run, Home default view, PgUp/PgDn cycle views.

## Plain MATLAB axes (no Viewer)

For headless runs or simple figures, configure `gca` directly and let
`phx.Body`/`phx.Simulation` default to it:

```matlab
clf; view(3); axis("equal"); grid("on"); camlight("headlight");
axis("manual");   % freeze limits so falling bodies don't rescale the view
```

After the run, plot logged data into a second figure (`figure(2); plot(...)`), as
the demos do — keep the simulation axes and the result plots separate.

## Related skills

- **phx-scene-basics** — bodies, shapes, `Simulation.step`, running headless.
- **phx-constraints-forces** — the elements whose `Force`/`Angle`/`Thrust` you typically log.
- **phx-engine-gotchas** — engine quirks, error IDs, tests.
- **phx-simulink** — exchange signals with a PHX scene from Simulink via the PhxModel block.
