function phxex_soil
% PHXEX_SOIL Excavator bucket scooping a pile of rocks
%
% A kinematic bucket (STL) animated by phx.Script digs through a heap of
% phx.shape.Rock bodies inside a walled bin.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Figure setup
    rng(0); % Set random seed for reproducibility
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-3, 6.6, 3], "DefaultCameraTarget", [0 0 0.6], "Texture", "Tiles");

    % Create static bodies representing the environment
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [6 3 0.06], "Color", [1 1 1], "Texture", "cells", "TextureBlend", 0.5});
    phx.Body(ax, "Type", "static", "Position", [3.06 0 0.36], "Shape", {"Box", "Size", [0.06 3 0.72], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [2.28 1.53 0.36], "Shape", {"Box", "Size", [1.5 0.06 0.72], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [2.28 -1.53 0.36], "Shape", {"Box", "Size", [1.5 0.06 0.72], "Color", [1 1 1]});

    % Random rocks of 0.36 m scattered over the digging area
    rocks = phx.assembly.scatter({"Rock", "Radius", 0.18}, 50, "Region", [1.2 2.4 1.2], "Spacing", 0.18, "Position", [2.4 0 0.12], "Color", [0.9 0.6 0.4], "RandomTint", 0.7);
    
    % Create a kinematic body (bucket of 1.2 m) for interaction
    exc = phx.Body(ax, "Type", "kinematic", "Position", [0 0 0.66], "Shape", {resdir+"Bucket.stl", "Scale", 0.012, "Centered", true, "Color", 0.4, "Envelope", "concave", "Style", "flat", "Texture", "stone", "TextureBlend", 0.2});

    % Add planar shadows
    phx.PlanarShadow([exc rocks], "Anchor", ground, "Position", [0 0 0.03]);
    
    % Detection zone
    zone = phx.Zone(ground, "Bodies", rocks, "Position", [-1.8 0 0.3], "Size", [2.4 3 0.6], "EnteredFcn", @zoneEnter);

    % Motion automation
    time = [0; 5; 10; 13.6; 18.6; 20.6; 25.6];
    position = [0 0 0.66; 0 0 0.66; 2.1 0 0.66; 2.1 0 1.2; -0.9 0 1.2; -0.9 0 1.2; -0.9 0 1.2];
    angle = [0 0 0; 0 0 0; 0 0 0; 0 0 pi; 0 0 pi; 0 pi/3 pi; 0 0 pi];
    phx.Script(exc, {"Position", time, position, "pchip"}, {"EulerAngles", time, angle});

    % Camera following the bucket
    phx.Camera(ground, exc, "PointA", [-3, 6.6, 3]);

    % Simulation; a margin of a few millimetres suits the rocks
    sim = phx.Simulation("EngineSettings", phx.engine.BulletSettings("Margin", 0.004)); % Initialize the simulation
    sim.step(25.6, 2560, 10); % Run the simulation and draw each 10th step
    delete(sim); % Clean up the simulation object

    % Number of rocks in a zone
    viewer.displayText("Total: "+zone.Count, "below");

    function zoneEnter(zone, ~)
        viewer.displayText("Rocks: "+repmat('•', [1, zone.Count]));
    end

end