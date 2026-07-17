function phxex_charges(drawSteps)
% PHXEX_CHARGES Charged cubes attracting and repelling in a monopole field
%
% Four cubes carry a positive or negative charge and interact through a single
% phx.Monopole, whose force field is visualised by an arrow grid.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        drawSteps (1, 1) double = 10
    end

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [1, 0, 100], "Texture", "defaultChecker"); % Set camera position and view mode

    % Physical model: an arena keeps the cubes in; its origin is lifted so
    % that the floor surface stays at z = 0.5, where the cubes rest
    parts = phx.assembly.arena("Size", [50 50 1.5], "Thickness", 1, "Position", [0 0 0.5], "Color", [1 1 1]);
    parts.floor.Friction = 0.01;
    set(parts.walls, "Color", [0.8 0.8 0.8]);

    % Create dynamic objects
    cubes(1) = phx.Body(ax, "Position", [-6 -6 3], "Shape", {"Box", "Size", 5, "Color", [0.5 0.5 1]});
    cubes(2) = phx.Body(ax, "Position", [18 18 3], "Shape", {"Box", "Size", 5, "Color", [0.5 0.5 1]});
    cubes(3) = phx.Body(ax, "Position", [-14 14 3], "Shape", {"Box", "Size", 5, "Color", [1 0.5 0.5]});
    cubes(4) = phx.Body(ax, "Position", [22 -22 3], "Shape", {"Box", "Size", 5, "Color", [1 0.5 0.5]});
    phx.Body(ax, "Position", [0 0 3], "Shape", {"Sphere", "Diameter", 5, "Color", 0.6, "Material", "metal"});

    % Add monopole interaction
    phx.Monopole(cubes, "Attractivity", -1, "Charge", [1 1 -1 -1]*1e4, ...
        "VectorFieldCenter", [0 0 3], "VectorFieldSize", [60 60 0], "VectorFieldStep", 3, ...
        "VectorLength", 6, "VectorSegments", 6, "Color", 0.5);
    
    % Simulation
    sim = phx.Simulation; % Create simulation based on current scene
    sim.step(60, 6000, drawSteps); % Step the simulation
    delete(sim); % Clean up the simulation object

end