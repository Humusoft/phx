function phxex_buggy(showGraphs)
% PHXEX_BUGGY Buggy driving downhill and colliding with cones
%
% This function initializes a figure, loads a buggy model from file and adds
% some bodies to represent a ground with cones. The buggy goes downhill and
% collides with some objects. Suspension spring deformation is recorded
% using a logger.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true 
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Setup figure
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [-21 4 -5], "DefaultCameraPosition", [-81 -47 15]);

    % Speed gauge
    speed = uigauge(gcf, "Limits", [0 5], "Position", [10 10 120 120]);

    % Create ground
    phx.Body(ax, "Type", "static", "Position", [0 0 -5], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 0.8]}, "EulerAngles", [0 -0.12 0]);
    phx.Body(ax, "Type", "static", "Position", [-50 0 -8], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 0.8]});

    % Load buggy model and show it in the current axes
    buggy = load("saved_buggy.mat");
    propagate(cell2mat(struct2cell(buggy)), "ParentAxes", ax);

    % Add logger to specific elements (springs)
    log = phx.Logger(buggy.chassis.Children(5:8), "Parameters", "Length", "Frequency", 100);

    % Create cones
    shpCone = phx.shape.Cone("Diameter", 2, "Height", 6, "Texture", resdir+"checker4.png", "TextureBlend", 0.25, "Color", [1 0.5 0]);
    phx.Body("Position", [-30 -7 0], "Shape", shpCone, "Mass", 1, "Inertia", 0.1);
    phx.Body("Position", [-40 -7 0], "Shape", shpCone, "Mass", 1, "Inertia", 0.1);
    phx.Body("Position", [-50 -7 0], "Shape", shpCone, "Mass", 1, "Inertia", 0.1);
    phx.Body("Type", "static", "Position", [-50 6.5 0], "Shape", {"Cylinder", "Diameter", 3, "Height", 14, "Color", [1 1 0.8]});

    % car = cell2mat(struct2cell(buggy));
    % car.groupTransform("Translation", [-3 0 0], "EulerAngles", [0 0 pi/6]);

    % Simulation loop
    sim = phx.Simulation;
    for i = 1:280
        sim.step(0.05, 50, 50);
        speed.Value = abs(buggy.wheelFR.AngularVelocity(2));
    end
    delete(sim);

    % Plot results
    if showGraphs
        figure(2);
        plot(log.Time, log.Data);
    end

end