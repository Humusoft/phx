function phxex_shapes
% PHXEX_SHAPES Gallery of the phx.shape types tipped off a tilting table
%
% Every shape class stands in its own column of a table, in two or three
% sizes or variants: boxes, spheres, cylinders, sharp and truncated cones,
% capsules, open tubes, rocks, imported STL meshes, extruded profiles and
% revolved profiles. The table then tilts towards the camera and the shapes
% leave it each in its own way - rolling, sliding, toppling or tumbling -
% down into a valley of a heightfield terrain, the last shape type.

%   Copyright 2026 HUMUSOFT s.r.o.

    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", ...
        "DefaultCameraPosition", [0 -11.5 6.2], "DefaultCameraTarget", [0 -2.2 1.1]);
    phx.base.Shape.resetColorOrder;

    % Landscape: a valley in front of a flat plateau that carries the table
    makeLandscape(ax);

    % Table hinged on an axle along x, its top at z = 2 m
    top = 2;
    table = makeTable(ax, top);

    % The gallery: one column per shape class, variants from front to back
    shapes = gallery(resdir);
    nCol = numel(shapes);
    xCol = ((1:nCol) - (nCol + 1)/2)*0.95;
    yRow = [-0.85 0 0.85];
    for c = 1:nCol
        for r = 1:numel(shapes{c}.Variants)
            v = shapes{c}.Variants{r};
            phx.Body(ax, "Position", [xCol(c) yRow(r) top + v.Lift], ...
                "EulerAngles", v.EulerAngles, "Shape", v.Shape);
        end
        text(table.Graphics, xCol(c), -1.45, 0.12, shapes{c}.Name, "Color", [0.15 0.15 0.15], ...
            "HorizontalAlignment", "center", "FontWeight", "bold", "FontSize", 10);
    end

    % Let the gallery settle, tilt the table towards the camera by 35 deg
    tilt = deg2rad(35);
    phx.Script(table, {"EulerAngles", [0 5 10]', [0 0 0; tilt 0 0; tilt 0 0;]});

    sim = phx.Simulation(ax);
    sim.step(10, 1000);
    delete(sim);
end

function makeLandscape(ax)
% Heightfield terrain: a flat plateau at 1 m for y > -1, falling into a
% valley along x at y = -4 and rising again at the far ends of x. The
% heights double as the texture.
    extent = [14 11];
    x = linspace(-extent(1)/2, extent(1)/2, 141);
    y = linspace(-8, 3, 111)';
    valley = (min(y, -1) + 4).^2/9;
    ends = 1.2*(x/7).^4;
    bumps = 0.05*sin(1.7*x).*cos(1.3*y).*(y < -1.5);
    H = max(valley, ends) + bumps;

    phx.Body(ax, "Type", "static", "Position", [0 -2.5 0], "Friction", 0.6, ...
        "Shape", {"Terrain", "Size", extent, "Height", H, "Texture", H, ...
                  "TextureColormap", summer(64), "Material", "matte"});
end

function table = makeTable(ax, top)
% Kinematic table plate turning about its centre line, two static legs and
% the axle between them
    h = 1.0;                                % plateau height
    plate = [9.9 3.2 0.2];
    zc = top - plate(3)/2;
    table = phx.Body(ax, "Type", "kinematic", "Position", [0 0 zc], "Friction", 0.5, ...
        "Shape", {"Box", "Size", plate, "Color", [0.85 0.8 0.7], ...
                  "Texture", phx.base.Shape.tiles([1024 512], [16 8])});
    xLeg = plate(1)/2 + 0.15;
    for x = [-xLeg xLeg]
        phx.Body(ax, "Type", "static", "Position", [x 0 (h + zc)/2], ...
            "Shape", {"Box", "Size", [0.2 0.4 zc - h + 0.15], "Color", 0.35, "Material", "metal"});
    end
    phx.Body(ax, "Type", "static", "Position", [0 0 zc], ...
        "Shape", {"Cylinder", "Axis", "x", "Diameter", 0.12, "Height", 2*xLeg + 0.3, ...
                  "Color", 0.35, "Material", "metal"});
end

function shapes = gallery(resdir)
% Shape columns, the variants from the front row to the back one. Each
% variant gives its shape, its orientation and the height of its centre
% above the table top (dropped a little where the resting height is not
% obvious). The envelopes are convex or primitive: nothing has to get inside
% the tubes, the ring or the bowl, and a concave mesh costs far more per step.
    v = @(shape, lift, euler) struct("Shape", {shape}, "Lift", lift, "EulerAngles", euler);
    flat = [0 0 0];

    shapes = {};

    shapes{end + 1} = struct("Name", "Box", "Variants", {{ ...
        v({"Box", "Size", [0.15 0.7 0.06]}, 0.04, flat), ...
        v({"Box", "Size", 0.4, "Color", [0.75 0.5 0.3], "Texture", "wood", "TextureBlend", 0.6}, 0.21, flat), ...
        v({"Box", "Size", [0.18 0.18 0.6]}, 0.31, flat)}});

    shapes{end + 1} = struct("Name", "Sphere", "Variants", {{ ...
        v({"Sphere", "Diameter", 0.2}, 0.11, flat), ...
        v({"Sphere", "Diameter", 0.35, "Texture", "checker", "TextureBlend", 0.3}, 0.18, flat), ...
        v({"Sphere", "Diameter", 0.55}, 0.28, flat)}});

    shapes{end + 1} = struct("Name", "Cylinder", "Variants", {{ ...
        v({"Cylinder", "Diameter", 0.5, "Height", 0.06}, 0.04, flat), ...
        v({"Cylinder", "Axis", "x", "Diameter", 0.25, "Height", 0.6, "Texture", "stripes", "TextureBlend", 0.3}, 0.13, flat), ...
        v({"Cylinder", "Diameter", 0.3, "Height", 0.5, "Style", "edged"}, 0.26, flat)}});

    shapes{end + 1} = struct("Name", "Cone", "Variants", {{ ...
        v({"Cone", "Axis", "x", "Diameter", [0.4 0.2], "Height", 0.45}, 0.22, flat), ...
        v({"Cone", "Diameter", [0.5 0.25], "Height", 0.3}, 0.16, flat), ...
        v({"Cone", "Diameter", 0.4, "Height", 0.55}, 0.29, flat)}});

    shapes{end + 1} = struct("Name", "Capsule", "Variants", {{ ...
        v({"Capsule", "Axis", "x", "Diameter", 0.18, "Height", 0.6}, 0.1, flat), ...
        v({"Capsule", "Axis", "y", "Diameter", 0.25, "Height", 0.5}, 0.13, flat), ...
        v({"Capsule", "Diameter", 0.35, "Height", 0.2, "Style", "edged"}, 0.19, flat)}});

    shapes{end + 1} = struct("Name", "Tube", "Variants", {{ ...
        v({"Tube", "Axis", "x", "InnerDiameter", 0.32, "WallThickness", 0.04, "Height", 0.3, "Segments", 48, ...
           "Envelope", "cylinder"}, 0.2, flat), ...
        v({"Tube", "InnerDiameter", 0.3, "WallThickness", 0.07, "Height", 0.08, "Segments", 48, ...
           "Envelope", "convex"}, 0.05, flat), ...
        v({"Tube", "InnerDiameter", [0.08 0.4], "WallThickness", 0.03, "Height", 0.35, "Segments", 48, ...
           "Envelope", "convex"}, 0.19, flat)}});

    rock = {"Rock", "Color", [0.6 0.57 0.52], "Texture", "stone", "TextureBlend", 0.6};
    shapes{end + 1} = struct("Name", "Rock", "Variants", {{ ...
        v([rock {"Diameter", 0.25, "Roundness", 0.9}], 0.15, flat), ...
        v([rock {"Diameter", 0.4, "Roundness", 0.6}], 0.23, flat), ...
        v([rock {"Diameter", 0.55, "Roundness", 0.4}], 0.32, flat)}});

    shapes{end + 1} = struct("Name", "Mesh", "Variants", {{ ...
        v({"Mesh", "Source", resdir + "rocket.stl", "Scale", 0.011, "Envelope", "cylinder", "Axis", "z"}, 0.12, [0 pi/2 0]), ...
        v({"Mesh", "Source", resdir + "rotor.stl", "Scale", 0.025, "Material", "metal", "Color", 0.7}, 0.12, flat), ...
        v({"Mesh", "Source", resdir + "cat.stl", "Scale", 0.01, "Details", 0.2, "Material", "glossy"}, 0.23, [0 0 pi/2])}});

    shapes{end + 1} = struct("Name", "Extrusion", "Variants", {{ ...
        v({"Extrusion", "Profile", starProfile(5, 0.22, 0.1), "Spine", [0 0 -0.04; 0 0 0.04]}, 0.05, flat), ...
        v({"Extrusion", "Profile", 0.04*[cos(th(16)) sin(th(16))], "Spine", arcSpine(0.18, 1.6*pi)}, 0.05, flat), ...
        v({"Extrusion", "Profile", [0 0; 0.12 0; 0.12 0.02; 0.02 0.02; 0.02 0.12; 0 0.12], ...
           "Spine", [-0.35 0 0; 0.35 0 0], "Material", "metal", "Color", 0.6}, 0.08, flat)}});

    shapes{end + 1} = struct("Name", "Revolution", "Variants", {{ ...
        v({"Revolution", "Axis", "x", "Profile", bottleProfile, "Envelope", "cones", ...
           "Color", [0.3 0.6 0.4], "Material", "glossy"}, 0.1, flat), ...
        v({"Revolution", "Profile", bowlProfile, "Segments", 48}, 0.09, flat), ...
        v({"Revolution", "Profile", pinProfile, "Envelope", "cones", "Color", 0.95}, 0.21, flat)}});
end

function t = th(n)
% n angles around a full circle, the last one short of 2*pi
    t = linspace(0, 2*pi, n + 1)';
    t(end) = [];
end

function P = starProfile(n, rOut, rIn)
% Star outline with n points
    a = th(2*n) + pi/2;
    r = repmat([rOut; rIn], n, 1);
    P = [r.*cos(a) r.*sin(a)];
end

function S = arcSpine(r, span)
% Horizontal circular arc around the origin, open by 2*pi - span
    a = linspace(-span/2, span/2, 40)';
    S = [r*cos(a) r*sin(a) zeros(size(a))];
end

function P = bottleProfile
% Bottle: [axis radius] from the bottom centre to the top of the neck
    P = [-0.22 0; -0.22 0.09; -0.2 0.1; 0.04 0.1; 0.1 0.05; 0.16 0.035; 0.22 0.035; 0.22 0];
end

function P = pinProfile
% Skittle with its wide belly low
    z = linspace(-0.2, 0.2, 21)';
    r = 0.05 + 0.07*exp(-((z + 0.07)/0.09).^2) + 0.03*exp(-((z - 0.15)/0.04).^2);
    P = [-0.2 0; z r; 0.2 0];
end

function P = bowlProfile
% Open bowl with a wall 2 cm thick: outside up from the base, inside down
    a = linspace(0, pi/2, 12)';
    outer = [-0.08 + 0.17*(1 - cos(a)) 0.17*sin(a) + 0.04];
    inner = flipud([-0.06 + 0.15*(1 - cos(a)) 0.15*sin(a) + 0.04]);
    P = [-0.08 0; outer; inner; -0.06 0];
end
