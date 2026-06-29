function phxex_minimal2
% PHXEX_MINIMAL2 Minimal scene - bodies falling onto a static floor
%
% A slightly larger companion to phxex_minimal1: a few default bodies drop onto
% a static box floor.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Clear the current figure, set 3D view, and configure lighting
    clf; view(3); axis("equal"); camlight("headlight"); grid("on");

    % Create bodies at specified positions
    phx.Body("Position", [0.6 0 2]);
    phx.Body("EulerAngles", [0 0 pi/3]);
    phx.Body("Type", "static", "Position", [0 0 -1], "Shape", {"Box", "Size", [5 3 0.1]});

    % Fix current axes limits
    axis("manual");

    % Initialize the simulation and run it for 100 steps
    sim = phx.Simulation;
    sim.step(3, 100, 1);

    % Clean up by deleting the simulation object
    delete(sim);

end