function phxex_buoyancy(duration, waves)
% PHXEX_BUOYANCY Floating bodies - buoyancy, righting moments and waves
%
% A handful of bodies with different shapes and densities is dropped into
% a pool of water. A single phx.Buoyancy element samples the interior of
% every body once with a regular grid of volume points; each step, the
% points below the water surface produce the buoyant force applied at the
% center of buoyancy, so the bodies not only float at the physically
% correct draft but also right themselves - a tilted crate rocks back
% upright because the center of buoyancy shifts under the submerged side.
%
% The light crate, buoy and log settle at drafts given by their density
% ratio, while the dense anchor sinks to the pool floor. With waves
% enabled, the water level is a time-varying LevelFunction and the
% floating bodies bob and drift on the swell.
%
% Input Arguments:
%     duration - simulated time in seconds (default 12)
%     waves    - enable waves on the water surface (default true)
%
% Example:
%     phxex_buoyancy             % default run with waves
%     phxex_buoyancy(20, false)  % longer run on calm water

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        duration (1, 1) double {mustBePositive} = 12
        waves (1, 1) logical = true
    end

    poolSize = 12;
    poolDepth = 2.5;
    rhoWater = 1000;

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 -0.5], ...
        "DefaultCameraPosition", [8 -10 4]);

    % Pool floor (the anchor will come to rest on it)
    phx.Body(ax, "Type", "static", "Position", [0 0 -poolDepth - 0.25], ...
        "Shape", {"Box", "Size", [poolSize poolSize 0.5], "Color", [0.75 0.7 0.6]}, ...
        "Friction", [0.6 0.1 0.1]);

    % Floating and sinking bodies: densities relative to water decide the
    % draft (400 -> 40 % submerged) or the fate (3000 -> sinks)
    crate = phx.Body(ax, "Position", [-2 -1.5 1.5], "EulerAngles", [0.4 0.3 0], ...
        "Shape", {"Box", "Size", [0.8 0.8 0.8], "Density", 400, ...
        "Style", "edged", "Color", [0.8 0.6 0.35]}, "Friction", [0.5 0.05 0.05]);
    buoy = phx.Body(ax, "Position", [1.8 -1.5 2], ...
        "Shape", {"Sphere", "Radius", 0.4, "Density", 300, "Color", [1 0.35 0.25]}, ...
        "Friction", [0.5 0.05 0.05]);
    timber = phx.Body(ax, "Position", [0 1.5 1.2], "EulerAngles", [0 0 0.6], ...
        "Shape", {"Capsule", "Radius", 0.25, "Height", 1.2, "Axis", "x", ...
        "Density", 600, "Color", [0.55 0.4 0.25]}, "Friction", [0.5 0.05 0.05]);
    anchor = phx.Body(ax, "Position", [2 1.8 0.8], ...
        "Shape", {"Box", "Size", [0.5 0.5 0.5], "Density", 3000, ...
        "Style", "edged", "Color", [0.35 0.35 0.4]}, "Friction", [0.6 0.1 0.1]);
    cat = phx.Body(ax, "Position", [2 1.8 1.8], ...
        "Shape", {"STL", "Source", "res/cat.stl", "Scale", 0.1, "Density", 800, ...
        "Color", [1 1 1]}, "Friction", [0.6 0.1 0.1]);

    bodies = [crate buoy timber anchor cat];
    names = ["crate" "buoy" "timber" "anchor" "cat"];

    % One water volume for all bodies; the level is either a calm plane or
    % a superposition of two travelling waves
    if waves
        level = @(x, y, t) 0.1*sin(1.4*x + 1.9*t) + 0.05*sin(2.3*y + 2.7*t);
    else
        level = [];
    end
    phx.Buoyancy(bodies, "Density", rhoWater, "Level", 0, "LevelFunction", level, ...
        "LinearDamping", 300, "AngularDamping", 100, ...
        "SurfaceSize", [poolSize poolSize], "SurfaceStep", 0.25);

    % Simulation - dt = 5 ms keeps the contacts and the bobbing stable
    sim = phx.Simulation(ax);
    subSteps = 10;
    dt = 0.005;
    while sim.Time < duration
        sim.step(dt*subSteps, subSteps, 5);
        viewer.displayText(sprintf("t = %4.1f s   crate z = %+.2f   anchor z = %+.2f", ...
            sim.Time, crate.Position(3), anchor.Position(3)));
        pause(0);
    end
    delete(sim);

    % Report the final drafts against the calm water level
    fprintf("Final heights of the body centers above the calm water level:\n");
    for i = 1:numel(bodies)
        fprintf("  %-6s  z = %+5.2f m\n", names(i), bodies(i).Position(3));
    end

end
