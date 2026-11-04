function phxex_screwconv(showGraphs)
% PHXEX_SCREWCONV Screw conveyor moving balls from a hopper
%
% The screw is a flat profile swept along a helix (phx.shape.Extrusion). It
% turns kinematically, picks up balls from a hopper and pushes them along a
% corridor; the axial force on the screw is logged.
%
% phxex_screwconv(false) skips the plot.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true 
    end
    
    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-2 2 2.5], "DefaultCameraTarget", [0 0 0.5]);

    % Ground
    phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [5 5 0.1], "Texture", "tiles"});

    % Extrusion shape defined with profile and spine
    t = linspace(0, 4*2*pi, 36*4)';
    r = 0.15;
    spine = [cos(t)*r, sin(t)*r, t*0.05];
    t = linspace(0, 2*pi, 8)';
    profile = [cos(t), sin(t)*0.2]*0.1;
    shape = phx.shape.Extrusion("Spine", spine, "Profile", profile, "Axis", "x", "Envelope", "concave", "Material", "glossy", "Color", [0.5 0.5 0.5]);
    screw = phx.Body(ax, "Type", "kinematic", "Position", [-0.5 0 0.3], "Shape", shape);

    % Walls
    phx.Body(ax, "Type", "static", "Position", [0 0.3 0.2], "Shape", {"Box", "Size", [2 0.05 0.4], "Color", 0.9});
    phx.Body(ax, "Type", "static", "Position", [0 -0.3 0.2], "Shape", {"Box", "Size", [2 0.05 0.4], "Color", 0.9});
    phx.Body(ax, "Type", "static", "Position", [1 0 0.2], "Shape", {"Box", "Size", [0.05 0.6 0.4], "Color", 0.9});
    phx.Body(ax, "Type", "static", "Position", [1 0 0.7], "Shape", {"Box", "Size", [0.05 0.6 0.6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});
    phx.Body(ax, "Type", "static", "Position", [0.7 0.3 0.7], "Shape", {"Box", "Size", [0.6 0.05 0.6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});
    phx.Body(ax, "Type", "static", "Position", [0.7 -0.3 0.7], "Shape", {"Box", "Size", [0.6 0.05 0.6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});
    phx.Body(ax, "Type", "static", "Position", [0.4 0 0.7], "Shape", {"Box", "Size", [0.05 0.6 0.6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});

    % Balls
    for j = -0.2:0.2:0.2
        for i = 1:30
            phx.Body(ax, "Position", [0.8 j 0.2+i*0.13], "Shape", {"Sphere", "Diameter", 0.13});
        end
    end

    % Automation script
    phx.Script(screw, {"EulerAngles", "[2*pi*t*1.5 0 0]"});

    % Force recording
    logger = phx.Logger(screw, "Parameters", "TotalForce", "Frequency", 100);

    % Simulation; a margin of a few millimetres suits the thin screw flight
    sim = phx.Simulation("EngineSettings", phx.engine.BulletSettings("Margin", 0.004));
    sim.step(5, 1500, 5);
    delete(sim);

    % Plot results
    if showGraphs
        clf(figure(2));
        plot(logger.Time, logger.Data(:, 1));
        grid("on");
        xlabel("Time (s)"); ylabel("Force (N)"); title("Axial force");
    end

end