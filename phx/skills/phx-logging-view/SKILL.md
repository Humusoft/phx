---
name: phx-logging-view
description: >
  Record and observe a PHX simulation — log object properties over time with
  phx.Logger, draw motion trails with phx.Trace, measure distances/velocities
  between bodies with phx.Measure, detect and count bodies in a region with
  phx.Zone, sense range/normal/body along rays with phx.Raycast, cast shadows
  onto a plane with phx.PlanarShadow (only when the user asks for them), set
  up the view with phx.Camera and the interactive phx.extra.Viewer, and style
  shapes (Color, Material, Style, Texture: built-in textures and their generators,
  image files and matrices, TextureBlend/TextureTransform). Use when capturing
  signals for plots, detecting events in a scene, sensing distances or line of
  sight, configuring how a PHX scene is displayed, or giving bodies colors,
  materials or textures.
---

# PHX logging and visualization

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
end-of-conveyor detection, "did it reach the target area" checks, and the fill
of a hopper or load bed (mass and its center).

```matlab
% Active zone: evaluated every step, callbacks fire on the transitions
zone = phx.Zone(ground, "Bodies", rocks, ...       % watch set (default: all
    "Position", [-30 0 5], ...                     %   non-static bodies)
    "Size", [40 50 10], ...                        % full box dims, anchor-local
    "EulerAngles", [0 0 0], ...
    "EnteredFcn", @(z, b) disp(z.Count), "ExitedFcn", []);

zone.Contents        % phx.Body array currently inside
zone.Count           % how many
zone.Mass            % total mass of the DYNAMIC bodies inside (kg)
zone.CenterOfMass    % their common center of mass, world [x y z]; NaN(1,3) if none
zone.EnteredCount    % cumulative throughput since the last pipeline rebuild
zone.ExitedCount

% Record the fill over time (Data columns: 1 = Mass, 2:4 = CenterOfMass)
L = phx.Logger(zone, "Parameters", ["Mass", "CenterOfMass"], "Frequency", 100);
```

`Mass`/`CenterOfMass` skip static and kinematic occupants (they still count in
`Count`), and a body counts whole or not at all, by whether its own center of
mass is inside.

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
Demos: `phxex_bowlfeeder` (counts parts at the outlet), `phxex_soil` (active zone + HUD).

## phx.Raycast — ray sensor

A set of rays anchored to a body (so they follow and rotate with it) reporting
what each one hits. Use it for range finders, lidar sweeps, ground/altitude
probes, line-of-sight checks and picking. Where a Zone asks "what is in this
box", a Raycast asks "what is in this direction, and how far".

```matlab
% Active sensor: cast every substep, results always current
n = 64;
th = linspace(-pi/4, pi/4, n);
lidar = phx.Raycast(scanner, ...
    "Origins", [0; 0; 0], ...                  % 3x1 = shared by all rays,
    "Ends", 30*[sin(th); zeros(1,n); -cos(th)]);  % or 3xN, one per ray

lidar.Points      % 3xN world hit points, NaN in the miss columns
lidar.Normals     % 3xN unit normals, facing the ray
lidar.Distances   % 1xN metres from the origin, NaN where nothing was hit
lidar.Hits        % 1xN logical mask of the rays that returned
lidar.Bodies      % phx.Body array of the hit bodies, numel == nnz(Hits)
lidar.Count       % number of rays
```

Rays run from a column of `Origins` to the matching column of `Ends`, both in the
**anchor's local frame** — a fan is usually one shared origin plus N ends. There
is no separate direction/range pair; scale the ends instead (`30*D`).

**Misses are `NaN`, not zeros**, so `plot3(P(1,:),P(2,:),P(3,:))` breaks over them
instead of drawing lines back to the origin. Each ray reports the *nearest* body;
a ray starting inside a body does not hit that body.

**Manual sensors cost nothing per substep.** Create with `"SimulationOrder",
"none"` and call `update(sensors)` for a reading — the right shape whenever the
sensor is sampled more slowly than the simulation steps. It takes an array:

```matlab
r = phx.Raycast(rover, "Ends", [0; 0; -5], "SimulationOrder", "none");
for k = 1:20
    sim.step(0.05, 10, -1);
    update(r);                      % one reading per 10 substeps
    if r.Hits, disp(r.Distances); end
end
```

Notes: the mode is read when the pipelines are built, so set `SimulationOrder` at
construction — assigning it later does nothing until a rebuild. Assigning `Ends` a
different number of columns voids the stored results (they report `NaN` until the
next cast). `Visible`/`Overlay` control the drawn rays and hit markers, which are
truncated at the hit point.

## phx.Measure — live measurement

`phx.Measure(bodyA, bodyB)` (optionally `phx.Measure(bodyA, bodyB, pointA, pointB)`,
or via `PointA`/`PointB`) reports mutual kinematics between two bodies live during the
simulation and draws the link into the scene. Dependent readouts, all loggable:
`Distance` (scalar), `Position` (vector between the points, global), `PositionInA` /
`PositionInB` (in the local frame of the respective body), and the matching
`Velocity` / `VelocityInA` / `VelocityInB`. `Overlay` draws it on top of geometry.

## phx.Camera — scriptable camera

`phx.Camera(bodyA, bodyB)` places the view camera at body A and aims it at body B
(offsets `PointA`/`PointB` in the local frames), every redraw. Use an invisible body as a
camera rig and move it with `phx.Script` for fly-throughs, or attach the camera to a
moving body to chase it. `TrackingLag` (s) smooths the motion with a first-order lag
(0 = rigid, `Inf` = static). `Viewer` defaults to the viewer of the figure body A is
drawn in, or to its axes.

**Recording a video.** Set `RecordFile` (MPEG-4) and optionally `RecordFPS` (default
30, frames per second of *simulation* time). Frames are grabbed on redraws, so the run
must redraw at least as often as the frame rate, otherwise you get one frame per redraw.
The default `"adaptive"` redraw mode skips redraws by timing, so record with
`RedrawMode = "full"` and an explicit `redrawStep`. Recording starts when the simulation
is built; the file is finalized only when the camera itself is deleted (`delete(sim)`
leaves it open):

```matlab
rig = phx.Body("Type", "kinematic", "Position", [6 -6 3], "Visible", false, "Collisions", false);
cam = phx.Camera(rig, ball, "TrackingLag", 0.2, "RecordFile", "drop.mp4");
sim = phx.Simulation("RedrawMode", "full");   % every requested frame, no skipping
sim.step(5, 500, 2);          % redraw every 2nd of 100 substeps/s = 50 redraws/s >= 30 fps
delete(cam);                  % closes drop.mp4
```

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
pan, zoom, object selection/drag, free-run, sky textures) and **the default container
for a PHX scene**; when plain axes or no graphics (`[]`) fit better is in
**phx-scene-basics**, "The scene shape". Open it with the `"clear"` verb and capture the
axes it returns:

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

All of that keeps working *while* `sim.step` runs under the default `RedrawMode`
(`"adaptive"`), but not under `"performance"`; what that means for callbacks fired in the
middle of a step is in **phx-scene-basics**, "phx.Simulation — lifecycle".

**`Texture`.** The equirectangular background of the sky sphere (shown in `ViewMode`
`"texture"`). Takes an image path *or* the name of a built-in texture bundled with the
toolbox — **`"sky"` (the default), `"nebula"`, `"hall"`, `"gradient"`, `"checker"`,
`"tiles"`** — matched case-insensitively. An unknown name errors
(`phx:Viewer:fileNotFound`) rather than falling back, so do not invent names.

The viewer is also what makes **shape** textures visible at all (see "Appearance of
shapes" below): in plain axes a textured shape silently shows just its `Color`.

**`Lighting`.** `"headlight"` (the default) puts one light at the camera, so whatever you
look at is lit — safe but flat, since the shading then carries no directional information.
`"studio"` uses three world-fixed lights that stay put while you orbit, so a body's shading
conveys its shape and its silhouette separates from the background; prefer it for
screenshots, videos and anything where form matters. `"none"` leaves the lighting to you.
The modes are deliberately exclusive: a headlight shines along the view axis, so mixing it
in would add a view-locked gradient that fights the studio key's directional one.

**Result plots** go into a separate figure (`figure(2); plot(...)`), as the demos do: the
viewer takes over its whole figure and is not designed to share it. Plain axes, when
the scene uses them, are kept apart from the result plots the same way.

## Appearance of shapes — Color, Material, Style, Texture

Every `phx.shape.*` takes these options, in the `"Shape"` cell of a body
(`phx.Body(ax, "Shape", {"Box", "Color", [1 0 0], "Texture", "wood"})`) or on a shape
object. `Color` is the base color, which `Material` shades and `Texture` blends with.

**Material and Style.** `Material` picks the shading preset — `"unlit"`, `"matte"`,
`"plastic"` (default), `"glossy"` or `"metal"`. Reach for `"unlit"` when the color
carries data (group, state, a mapped value) rather than shape: it renders the authored
`Color` exactly and ignores the lights, so nothing shades the encoding away. `Style` is
independent of it: `"smooth"` (default), `"edged"`, `"flat"` or `"wireframe"`.

**Textures.** `Texture` takes the name of a built-in texture, an image file path, or an
image matrix. The ten built-in textures are grayscale patterns (matched
case-insensitively):

| Patterns | Surfaces |
|---|---|
| `"arrows"` (chevrons pointing right), `"cells"`, `"checker"` (2×2 squares), `"circles"` (concentric rings), `"scratches"`, `"stripes"` (vertical), `"tiles"` (8×8 light tiles), `"xor"` (nested squares) | `"wood"`, `"stone"` (cloudy noise) |

`TextureBlend` (0–1, default 1) mixes the texture with `Color`, so a light blend keeps
the body's own color while giving it surface detail:

```matlab
crate = phx.shape.Box("Size", 0.4, "Color", [0.75 0.5 0.3], "Texture", "wood", "TextureBlend", 0.6);
ball  = phx.shape.Sphere("Diameter", 1, "Color", [0.9 0.3 0.3], ...
                         "Texture", "checker", "TextureBlend", 0.3);
```

An unknown name errors (`phx:Shape:fileNotFound`) instead of falling back, and the shape
names are a *different* set from the viewer's sky textures (`"nebula"` is not a shape
texture). `"metal"` is the old name of `"scratches"`: it still works but warns
(`phx:Shape:renamedTexture`), so write `"scratches"`.

Where textures help — use them deliberately, not on everything:
- **Floors and platforms:** `"tiles"` on the ground box gives the scene scale and makes
  sliding and rolling bodies readable against it (it is light, so no blend is needed).
- **Rotating parts:** wheels, rotors, shafts, rollers and balls look static when
  uniformly colored. A low blend (0.2–0.5) of `"stripes"` or `"checker"` makes the
  rotation visible; `"stripes"` suits cylinders and wheels, `"checker"` spheres.
- **A few featured objects:** `"wood"` on crates and planks, `"stone"` on boulders and
  rocks, `"scratches"` on machine parts, `"arrows"` on conveyor belts.
- **Not on large groups.** Leave bulk material, granular piles, debris, scattered rocks
  and similar crowds of bodies plain colored (vary `Color` instead): a texture per body
  costs memory and draw time and adds only visual noise at that scale.

**Texture generators.** Each built-in texture is a static function that returns its
image as `uint8`, `pattern = phx.base.Shape.name(resolution, ...)` with `resolution`
as `[width height]` in pixels (default `[512 512]`). Prefer the plain name; call the
function only when the default does not fit:
- the pattern scale must match the object — more `tiles` on a large floor, wider
  `stripes`, more `wood` rings on a long plank;
- the face is far from square — a `resolution` with its aspect ratio keeps the pattern
  undistorted;
- a few objects should differ — another `seed` of `cells`, `scratches`, `stone`,
  `tiles` or `wood`.

| Function | Further arguments (default) |
|---|---|
| `arrows`, `stripes`, `circles` | `width` of one band/ring in pixels (64, 128, 32) |
| `checker` | `count` [across down] ([2 2]) |
| `tiles` | `count` ([8 8], must divide `resolution`), `base` joint lightness (0.8), `seed` |
| `cells` | `density` number of cells (40), `seed` |
| `stone` | `smoothness` (1.8, higher = larger, softer blotches), `seed` |
| `scratches` | `seed` |
| `wood` | `rings` (7), `seed` |
| `xor` | — |

```matlab
floor = phx.shape.Box("Size", [20 20 0.2], "Texture", phx.base.Shape.tiles([512 512], [16 16]));
plank = phx.shape.Box("Size", [2 0.3 0.05], "Color", [0.75 0.5 0.3], "TextureBlend", 0.6, ...
                      "Texture", phx.base.Shape.wood([1024 256], 12));
```

Generating costs up to ~15 ms (`"wood"`), and a name or generator call inside a
per-body `{"Box", ..., "Texture", ...}` spec generates the image again for every body.
When many bodies share a textured shape, create the shape object once and pass that.

`Texture` also takes an image matrix: M×N×3 RGB (`uint8`, or 0–1) or M×N gray levels.
`uint8` gray levels are used as they are (0 black, 255 white); data of any other class
are scaled to their full range, so `peaks(200)` works directly. `TextureColormap` (N×3,
default empty = gray) maps an M×N matrix to colors instead. Name-value order does not
matter:

```matlab
slab = phx.shape.Box("Size", [2 2 0.5], "Color", [0.9 0.5 0.2], ...
                     "Texture", peaks(200), "TextureBlend", 0.6);   % tinted gray pattern
trn  = phx.shape.Terrain("Height", H, "Texture", H, "TextureColormap", parula(256));
```

**Own patterns.** When neither a built-in texture nor its parameters give the pattern
needed (markings, a grid of a given pitch, a logo, data), build the matrix yourself or
load an image file. A row and a column vector with implicit expansion are enough for
most patterns. Make it `uint8` when its gray levels matter (a light, low-contrast
pattern); a `double` result is stretched to the full range anyway. Combine generator
outputs in `double`, since `uint8` arithmetic rounds and saturates every step. A
texture never repeats — one image spans the whole face — so the number of repetitions
is part of the matrix:

```matlab
x = 0:511; y = (0:511)';
pitch = uint8(255 - 60*(mod(x, 64) < 4 | mod(y, 64) < 4));   % light plate, darker grid every 64 px
mix   = 0.7*double(phx.base.Shape.stone) + 0.3*double(phx.base.Shape.cells);
plate = phx.shape.Box("Size", [4 4 0.1], "Texture", pitch);
rock  = phx.shape.Sphere("Diameter", 1, "Color", [0.5 0.45 0.4], "TextureBlend", 0.6, "Texture", mix);
```

**Resolution.** 512×512, the generator default, suits most objects. Small or numerous
objects do with 128–256 pixels; a large single object seen up close, typically the
floor, may want 1024–2048. Every textured body gets its own copy of the image on the GPU
at 4 bytes per pixel (1 MB at 512×512, 16 MB at 2048×2048), so do not go beyond what
the object needs.

`TextureTransform` edits the image before the blend: a string array of operations applied
left to right — `"rot90"`/`"rot180"`/`"rot270"` (counterclockwise, like `rot90`),
`"fliplr"`, `"flipud"`, `"grayscale"` (luma, like `rgb2gray`), `"invert"` and `"opaque"`
(drops the alpha channel). Use it to turn or mirror a texture on all faces at once, and
`"grayscale"` before a blend to tint a colored photo with `Color` (the built-in textures
are already grayscale). An image with any transparent pixel switches the draw path to
transparency, which turns antialiasing off; `"opaque"` brings it back when the alpha is
unwanted. On a `Mesh`, `TextureBlend` and `TextureTransform` affect only the shape's own
`Texture`, never textures from OBJ materials.

```matlab
panel = phx.shape.Box("Size", 1, "Texture", "photo.jpg", "Color", [0.2 0.4 0.9], ...
                      "TextureTransform", ["grayscale" "fliplr"], "TextureBlend", 0.6);
```

> **Textures render only in `phx.extra.Viewer` axes.** Only the viewer's axes take the
> fast world-primitive draw path that carries texture data; in plain axes (and whenever
> `ForcePatch` is true) the shape falls back to a `patch` and the texture is silently
> ignored — nothing errors, the surface just shows its `Color`. So if you set a texture,
> build the scene into the viewer's `ax`.

**Texture orientation.** An image reads unmirrored from outside. `Sphere` takes a standard
2:1 equirectangular map (top row at the +z pole, left/right edges on the +x meridian,
longitude anticlockwise about +z). A `Box` side face shows the image upright (top
towards +z), the top face with the image top towards +y. Revolved shapes put the image
top at the end of the profile, `Extrusion` at the end of the spine. `Rock` projects the
image onto each face along the axis closest to its normal (the whole image spans the
`Diameter`), so a noise-like texture such as the built-in `"stone"` suits it best. `Terrain` instead
follows its `Height` matrix (image row 1 along the lowest y), so `Height` itself set as
`Texture` lines up with the heights; a texture matrix, like an image, runs its columns
along u and rows along v. `Mesh` `UV` data: u along the image
columns, v *down* the rows ((0,0) = top-left); OBJ/PLY files (v up) are converted on load.
A `Mesh` without UVs (any STL, or OBJ/PLY/data without them) box-projects `Texture` like
`Rock`: the whole image spans the largest extent of the mesh, with no repeat and visible
jumps where neighbouring faces change axis, so prefer a noise-like texture there.

## Related skills

- **phx-scene-basics** — bodies, shapes, `Simulation.step`, running headless.
- **phx-constraints-forces** — the elements whose `Force`/`Angle`/`Thrust` you typically
  log, plus `phx.Function` for per-step logic reacting to what you observe.
- **phx-engine-gotchas** — accuracy diagnostics, engine settings, known quirks, error IDs.
- **phx-simulink** — exchange signals with a PHX scene from Simulink via the PhxModel block.
- **phx-upgrade-from-1x** — converting code, saved scenes and Simulink models written for PHX 1.x.
