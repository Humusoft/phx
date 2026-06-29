function phxex_load
% PHXEX_LOAD Function to load and simulate a saved catastrophe scenario
%
% This function sets up the 3D view and loads a simulation from a file,
% propagating the simulation and stepping through it for a specified duration.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Clear the current figure, set 3D view, and configure lighting
    [~, ax] = phx.extra.Viewer("clear", "ViewMode", "axis");
    xlim([-30 30]); ylim([-30 30]); zlim([-30 30]);

    % Load the simulation from the specified .mat file
    sim = phx.Simulation("saved_catastrophy.mat");

    % Set a new axes as a target for visualization of simulation objects
    sim.propagate("ParentAxes", ax);

    % Simulate the scene for 5 seconds
    sim.step(5, 500, 5);

    % Clean up by deleting the simulation object
    delete(sim);

end