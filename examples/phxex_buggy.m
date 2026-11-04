function phxex_buggy(showGraphs)
% PHXEX_BUGGY Buggy driving downhill and colliding with cones
%
% A buggy loaded from a saved scene rolls down a slope into a row of cones.
% A gauge shows the wheel speed and a logger records the lengths of the
% suspension springs.
%
% phxex_buggy(false) skips the plot.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true 
    end

    % Setup figure
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [-2.1 0.4 -0.5], "DefaultCameraPosition", [-8.1 -4.7 1.5], "Lighting", "studio");

    % Speed gauge
    speed = uigauge(gcf, "Limits", [0 10], "Position", [10 10 120 120]);

    % Create ground
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], "Shape", {"Box", "Size", [5 5 0.1], "Texture", "tiles", "Material", "metal"}, "EulerAngles", [0 -0.12 0]);
    phx.Body(ax, "Type", "static", "Position", [-5 0 -0.8], "Shape", {"Box", "Size", [5 5 0.1], "Texture", "tiles", "Material", "metal"});

    % Load buggy model and show it in the current axes
    buggy = load("saved_buggy.mat");
    propagate(cell2mat(struct2cell(buggy)), "ParentAxes", ax);

    % Add logger to specific elements (springs)
    log = phx.Logger(buggy.chassis.Children(5:8), "Parameters", "Length", "Frequency", 100);

    % Create cones and obstacle
    shpCone = phx.shape.Cone("Diameter", 0.2, "Height", 0.6, "Texture", "xor", "TextureBlend", 0.25, "Color", [1 0.5 0]);
    phx.Body("Position", [-3 -0.7 0], "Shape", shpCone, "Mass", 1, "Inertia", 0.1);
    phx.Body("Position", [-4 -0.7 0], "Shape", shpCone, "Mass", 1, "Inertia", 0.1);
    phx.Body("Position", [-5 -0.7 0], "Shape", shpCone, "Mass", 1, "Inertia", 0.1);
    phx.Body("Type", "static", "Position", [-5 0.65 0], "Shape", {"Cylinder", "Diameter", 0.3, "Height", 1.4, "Color", [1 0.8 0.6], "Texture", "stone", "TextureBlend", 0.6});

    % Simulation loop; contact tolerances for gaps of about 10 cm keep the
    % wheels clear of the body
    sim = phx.Simulation("EngineSettings", phx.engine.BulletSettings("LengthScale", 0.1));
    for i = 1:120
        sim.step(0.05, 50, 50);
        speed.Value = abs(buggy.wheelFR.AngularVelocity(2));
    end
    delete(sim);

    % Plot results
    if showGraphs
        figure(2);
        plot(log.Time, log.Data);
        title("Spring lengths");
        legend(get([log.Parents{:}], "Name"))
        grid("on");
    end

end