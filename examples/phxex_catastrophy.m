function phxex_catastrophy
% PHXEX_CATASTROPHY Random boxes and STL cats tumbling onto the ground
%
% Randomly sized boxes and STL cats fall onto a static ground; phx.Trace
% draws the path of each falling box.

%   Copyright 2026 HUMUSOFT s.r.o.

    rng(0); % Set random seed for reproducibility

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Set up the viewer with a default camera position and view mode
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [40, -60, 15], "Texture", "defaultNebula");
    phx.base.ShapeMesh.resetColorOrder; % Reset color order for shapes

    % Create a static ground body
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 -1], "Shape", {"Box", "Size", [50 50 1]});

    % Create dynamic boxes with random colors and sizes and attach traces
    for i = 1:40
        clr = rand(1, 3); % Generate a random color
        scl = rand(1, 3)*5; % Generate a random size
        boxes(i) = phx.Body(ax, "Position", [0 0 10+i*2], "Shape", {"Box", "Size", scl, "Color", clr});
        phx.Trace(boxes(i), "TracePoints", 50, "Point", scl/2, "Color", clr);
    end

    % Load a cat shape from an STL file and create scattered bodies above the ground
    catShape = phx.shape.STL("Source", resdir+"cat.stl", "Scale", 0.1, "Details", 0.5, "Color", [1 1 1], "Envelope", "convex");
    cats = phx.assembly.scatter(catShape, 10, "Region", [40 40 0], "Spacing", 6, "Position", [0 0 5]);

    % Initialize the simulation with the ground, boxes, and cats
    sim = phx.Simulation([ground boxes cats]);

    % Step the simulation forward and update the figure each 10 iterations
    sim.step(20, 2000, 10);

    % Clean up the simulation object
    delete(sim); 
        
end