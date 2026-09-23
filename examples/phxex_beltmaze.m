function phxex_beltmaze
% PHXEX_BELTMAZE Routes parcels through a maze of conveyor belts
%
% Eight static belts are joined into a sorting maze: a two-lane feeder
% that splits at a T junction, two branches that turn through L corners at
% different speeds, and a second T where they merge into a shared outfeed.
% Nothing in the scene moves except the parcels - every belt is a static
% box with a SurfaceVelocity, so contacts with it are solved as if its
% surface were sliding. The colour of a belt is its speed on the turbo
% colormap, and the arrow texture points along the local x axis of the
% drive.
%
% See also phx.Body/SurfaceVelocity, phxex_conveyors, phx.extra.Viewer

%   Copyright 2026 HUMUSOFT s.r.o.

    nParcels = 12;
    beltFriction = [1.0 0 0];

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Belt segments, each built along its own +x axis so that the drive, the
    % arrows and the length all follow the same direction.
    %       centre x    y    yaw   length  width  speed  sweep
    segs = [ -3.50   0.00     0     7.00   3.20   1.2   % feeder, two lanes
              0.80   1.60    90     3.20   1.60   1.5   % T junction, left branch
              0.80  -1.60   -90     3.20   1.60   0.7   % T junction, right branch
              4.00   4.00     0     8.00   1.60   1.1   % long run, left
              8.80   2.80   -90     4.00   1.60   1.3   % spine, coming down
              4.00  -4.00     0     8.00   1.60   2.2   % long run, right (the express)
              8.80  -2.80    90     4.00   1.60   0.9   % spine, coming up
             11.00   0.00     0     6.00   1.60   1.8]; % outfeed

    % Figure setup
    figure(1);
    phx.extra.Viewer("clear", "Texture", "tiles", "Lighting", "studio", ...
        "DefaultCameraTarget", [0.8 -0.3 0], "DefaultCameraPosition", [-10 -13 11]);

    % Belts, coloured by speed
    speed = segs(:, 6);
    cmap = nebula(256);
    for i = 1:size(segs, 1)
        s = segs(i, :);
        shade = round(1 + 255*(s(6) - min(speed))/(max(speed) - min(speed)));
        phx.Body("Type", "static", "Position", [s(1) s(2) -0.1], ...
            "EulerAngles", [0 0 s(3)*pi/180], "Friction", beltFriction, ...
            "SurfaceVelocity", [s(6) 0 0], ...
            "Shape", {"Box", "Size", [s(4) s(5) 0.2], "Color", cmap(shade, :), ...
            "Texture", resdir+"arrows.png", "TextureBlend", 0.5});
    end

    % Parcels, dropped onto the feeder in a repeatable pseudo-random spread.
    % The lane sets the route, the position along the belt sets the order.
    rng(7);
    colors = summer(nParcels);
    lane = sign(rand(nParcels, 1) - 0.5);
    for i = 1:nParcels
        phx.Body("Position", [-6.6 + 4.8*rand, lane(i)*(0.45 + 0.85*rand), 0.5], ...
            "Shape", {"Box", "Size", rand(1, 3) + 0.2, "Color", colors(i, :), "Texture", "wood", "TextureBlend", 0.5});
    end

    % Speed legend
    colormap(gca, cmap);
    clim(gca, [min(speed) max(speed)]);
    cb = colorbar(gca, "Color", "w", "FontSize", 14);
    cb.Label.String = "belt speed (m/s)";
    cb.Label.FontSize = 14;

    % Run the simulation
    dt = 0.005;
    sim = phx.Simulation;
    sim.step(24, 24/dt, 2);

    % Clean up by deleting the simulation object
    delete(sim);

end