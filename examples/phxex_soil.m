function phxex_soil
% PHXEX_SOIL Excavator bucket scooping a pile of rocks
%
% A kinematic bucket (STL) animated by phx.Script digs through a heap of
% phx.shape.Rock bodies inside a walled bin.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    rng(0); % Set random seed for reproducibility
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-50, 110, 50], "DefaultCameraTarget", [0 0 10], "Texture", "defaultPlane");

    % Create static bodies representing the environment
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [100 50 1], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [51 0 6], "Shape", {"Box", "Size", [1 50 12], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [38 25.5 6], "Shape", {"Box", "Size", [25 1 12], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [38 -25.5 6], "Shape", {"Box", "Size", [25 1 12], "Color", [1 1 1]});

    % Random rocks scattered over the digging area
    rocks = phx.assembly.scatter({"Rock", "Radius", 3}, 50, "Region", [20 40 20], ...
        "Spacing", 3, "Position", [40 0 2], "Color", (rand(50, 1) + [1 0.5 0])/2);
    
    % Create a kinematic body (bucket) for interaction
    exc = phx.Body(ax, "Type", "kinematic", "Position", [0 0 11], "Shape", {resdir+"Bucket.stl", "Scale", 0.2, "Centered", true, "Color", 0.4, "Envelope", "concave", "Style", "flat"});
    
    % Detection zone
    zone = phx.Zone(ground, "Bodies", rocks, "Position", [-30 0 5], "Size", [40 50 10], "EnteredFcn", @zoneEnter);

    % Motion automation
    time = [0; 5; 10; 13.6; 18.6; 20.6; 25.6];
    position = [0 0 11; 0 0 11; 35 0 11; 35 0 20; -15 0 20; -15 0 20; -15 0 20];
    angle = [0 0 0; 0 0 0; 0 0 0; 0 0 pi; 0 0 pi; 0 pi/3 pi; 0 0 pi];
    phx.Script(exc, {"Position", time, position, "pchip"}, {"EulerAngles", time, angle});

    % Camera following the bucket
    phx.Camera(ground, exc, "PointA", [-50, 110, 50]);

    % Simulation
    sim = phx.Simulation; % Initialize the simulation
    sim.step(25.6, 2560, 10); % Run the simulation and draw each 10th step
    delete(sim); % Clean up the simulation object

    % Number of rocks in a zone
    viewer.displayText("Total: "+zone.Count, "below");

    function zoneEnter(zone, ~)
        viewer.displayText("Rocks: "+repmat('•', [1, zone.Count]));
    end

end