function phxex_shapes
% PHXEX_SHAPES Gallery of the built-in phx.shape primitives
%
% Drops a box, sphere, cylinder, cone, capsule and an STL mesh onto a floor to
% show the available shape types and their draw styles.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    clf; view(3); axis("equal"); camlight("headlight"); grid("on");

    % Reset color order for shapes
    phx.base.ShapeMesh.resetColorOrder;

    % Add a static box shape to the simulation
    phx.Body("Type", "static", "Position", [1 0 -0.1], "Shape", {"Box", "Size", [5 5 0.2], "Color", [1 1 1]});

    % Add a sphere shape to the simulation
    phx.Body("Position", [0.5 0 2], "Shape", {"Sphere", "Style", "wireframe"});

    % Add a cylinder shape to the simulation
    phx.Body("Position", [0 0 0.5], "Shape", {"Cylinder", "Axis", "y", "Style", "edged"});

    % Add a cone shape to the simulation
    phx.Body("Position", [2.5 0 0.5], "Shape", {"Cone", "Axis", "z"});

    % Add a capsule shape to the simulation
    phx.Body("Position", [2.5 0 2], "Shape", {"Capsule", "Axis", "z", "Style", "edged"});

    % Add an STL model to the simulation
    phx.Body("Position", [1 -1.5 1], "Shape", {"STL", "Source", resdir+"cat.stl", "Scale", 0.025, "Details", 0.2, "Material", "shiny"});

    % Fix current axes limits
    axis("manual");

    % Create and run the simulation for 4 seconds with 400 steps
    sim = phx.Simulation;
    sim.step(4, 400, 1);

    % Clean up by deleting the simulation object
    delete(sim);

end