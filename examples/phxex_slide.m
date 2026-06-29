function phxex_slide
% PHXEX_SLIDE Demonstrate profile extrusion
%
% This function creates a slide by extruding a profile along a curve.
% Balls are then dropped onto the slide, and their movement is
% simulated.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [3 5 4], "DefaultCameraPosition", [-8 32 24]);

    % Physical model
    phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 1]});

    % Extrusion shape defined with profile and spine
    t = linspace(0, 2*pi, 36)';
    spine = [-sin(t).*t, t*5, 2 + t];
    t = linspace(0, 2*pi, 30)';
    profile = [cos(t)*2, sin(t)*0.5+cos(t*2)+1.2];
    shape = phx.shape.Extrusion("Spine", spine, "Profile", profile, "Envelope", "concave", "Material", "matte", "Texture", resdir+"tiles.jpg");
    phx.Body(ax, "Type", "static", "Position", [0 -10 1], "Shape", shape);

    % Ball
    ball1 = phx.Body(ax, "Position", [0 20 15], "Shape", {"Sphere", "Diameter", 2, "Color", [0.5 0.5 1]});
    ball2 = phx.Body(ax, "Position", [3 18 15], "Shape", {"Sphere", "Diameter", 2, "Color", [0.2 0.8 1]});

    % Obstacle
    phx.Body(ax, "Position", [0 -20 4], "Shape", {"Box", "Size", [4 2 2], "Color", [1 0.2 0.1]});

    % Ball traces
    phx.Trace(ball1, "TracePoints", 500, "Color", (1 + ball1.Color)/2);
    phx.Trace(ball2, "TracePoints", 500, "Color", (1 + ball2.Color)/2);

    % Simulation
    sim = phx.Simulation;
    sim.step(15, 1500, 3);
    delete(sim);

end