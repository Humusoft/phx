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
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-50, 110, 50], "DefaultCameraTarget", [0 0 10]);

    % Create static bodies representing the environment
    phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [100 50 1], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [51 0 6], "Shape", {"Box", "Size", [1 50 12], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [38 25.5 6], "Shape", {"Box", "Size", [25 1 12], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [38 -25.5 6], "Shape", {"Box", "Size", [25 1 12], "Color", [1 1 1]});

    % Create random rocks in the environment
    for i = 1:50
        phx.Body(ax, "Position", [rand*20+30 rand*40-20 2+i*0.4], "Shape", {"Rock", "Radius", 3, "Color", (rand + [1 0.5 0])/2});
    end
    
    % Create a kinematic body (bucket) for interaction
    exc = phx.Body(ax, "Type", "kinematic", "Position", [0 0 11], "Shape", {resdir+"Bucket.stl", "Scale", 0.2, "Centered", true, "Color", 0.4, "Envelope", "concave", "Style", "flat"});
    
    % Motion automation
    time = [0; 5; 10; 13.6; 18.6; 20.6; 25.6];
    position = [0 0 11; 0 0 11; 35 0 11; 35 0 20; -15 0 20; -15 0 20; -15 0 20];
    angle = [0 0 0; 0 0 0; 0 0 0; 0 0 pi; 0 0 pi; 0 pi/3 pi; 0 0 pi];
    phx.Script(exc, {"Position", time, position, "pchip"}, {"EulerAngles", time, angle});

    % Simulation
    sim = phx.Simulation; % Initialize the simulation
    sim.step(25.6, 2560, 10); % Run the simulation and draw each 10th step
    delete(sim); % Clean up the simulation object

end