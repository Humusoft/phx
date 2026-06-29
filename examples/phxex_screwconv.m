function phxex_screwconv(showGraphs)
% PHXEX_SCREWCONV Demonstrate screw conveyor
%
% This feature demonstrates the use of profile extrusion to create a screw
% conveyor that picks up balls from the hopper and moves them through the
% corridor.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true 
    end
    
    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-20 20 25], "DefaultCameraTarget", [0 0 5]);

    % Ground
    phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 1]});

    % Extrusion shape defined with profile and spine
    t = linspace(0, 4*2*pi, 36*4)';
    r = 1.5;
    spine = [cos(t)*r, sin(t)*r, t*0.5];
    t = linspace(0, 2*pi, 8)';
    profile = [cos(t), sin(t)*0.2];
    shape = phx.shape.Extrusion("Spine", spine, "Profile", profile, "Axis", "x", "Envelope", "concave", "Material", "shiny", "Color", [0.5 0.5 0.5]);
    screw = phx.Body(ax, "Type", "kinematic", "Position", [-5 0 3.0], "Shape", shape);

    % Walls
    phx.Body(ax, "Type", "static", "Position", [0 3 2], "Shape", {"Box", "Size", [20 0.5 4], "Color", 0.9});
    phx.Body(ax, "Type", "static", "Position", [0 -3 2], "Shape", {"Box", "Size", [20 0.5 4], "Color", 0.9});
    phx.Body(ax, "Type", "static", "Position", [10 0 2], "Shape", {"Box", "Size", [0.5 6 4], "Color", 0.9});
    phx.Body(ax, "Type", "static", "Position", [10 0 7], "Shape", {"Box", "Size", [0.5 6 6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});
    phx.Body(ax, "Type", "static", "Position", [7 3 7], "Shape", {"Box", "Size", [6 0.5 6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});
    phx.Body(ax, "Type", "static", "Position", [7 -3 7], "Shape", {"Box", "Size", [6 0.5 6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});
    phx.Body(ax, "Type", "static", "Position", [4 0 7], "Shape", {"Box", "Size", [0.5 6 6], "Color", [1 1 0.8], "Style", "wireframe", "ForcePatch", true});

    % Balls
    for j = -2:2:2
        for i = 1:30
            phx.Body(ax, "Position", [8 j 2+i*1.3], "Shape", {"Sphere", "Diameter", 1.3});
        end
    end

    % Automation script
    phx.Script(screw, {"EulerAngles", "[2*pi*t*0.5 0 0]"});

    % Force recording
    logger = phx.Logger(screw, "Parameters", "TotalForce", "Frequency", 100);

    % Simulation
    sim = phx.Simulation;
    sim.step(15, 1500, 5);
    delete(sim);

    % Plot results
    if showGraphs
        figure(2);
        plot(logger.Time, logger.Data(:, 1));
        grid("on");
        xlabel("Time (s)"); ylabel("Force (N)"); title("Axial force");
    end

end