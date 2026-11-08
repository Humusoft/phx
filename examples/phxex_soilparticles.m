function phxex_soilparticles
% PHXEX_SOILPARTICLES Excavator bucket scooping gravel made of particles
%
% phxex_soil with the heap of rocks replaced by a phx.Particles group that
% fills the walled bin.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory of the examples
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Figure setup
    rng(0); % Set random seed for reproducibility
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-3, 6.6, 3], "DefaultCameraTarget", [0 0 0.6], "Texture", "Tiles");

    % Create static bodies representing the environment: a ground slab and a
    % bin of 1.5 x 3 m with walls 0.72 m high
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [6 3 0.06], "Color", [1 1 1], "Texture", "cells", "TextureBlend", 0.5});
    phx.Body(ax, "Type", "static", "Position", [3.06 0 0.36], "Shape", {"Box", "Size", [0.06 3 0.72], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [2.28 1.53 0.36], "Shape", {"Box", "Size", [1.5 0.06 0.72], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [2.28 -1.53 0.36], "Shape", {"Box", "Size", [1.5 0.06 0.72], "Color", [1 1 1]});

    % Gravel of 78 mm stones filling the bin: layers of a lattice, each
    % shifted at random so that the heap settles instead of standing as a block
    r = 0.039;
    s = 0.072;
    [x, y, z] = ndgrid(1.59:s:2.94, -1.38:s:1.38, 0.03 + r + (0:4)*s*2);
    shift = s*(rand(2, size(z, 3)) - 0.5);
    x = x + reshape(shift(1, :), 1, 1, []);
    y = y + reshape(shift(2, :), 1, 1, []);
    gravel = phx.Particles(ax, "Positions", [x(:) y(:) z(:)], "Radius", r, "Density", 2600, ...
        "Color", [0.9 0.6 0.4], "RandomTint", 0.5, "MarkerSize", 18, "Sleeping", false, "Friction", 0.4);

    % Create a kinematic body (bucket of 1.2 m) for interaction
    exc = phx.Body(ax, "Type", "kinematic", "Position", [0 0 0.66], "Friction", 0.1, "Shape", {resdir+"Bucket.stl", "Scale", 0.012, "Centered", true, "Color", 0.4, "Envelope", "concave", "Style", "flat", "Texture", "stone", "TextureBlend", 0.2});

    % % Add planar shadows
    % phx.PlanarShadow(exc, "Anchor", ground, "Position", [0 0 0.03]);

    % Motion automation
    time = [0; 5; 10; 13.6; 18.6; 20.6; 25.6];
    position = [0 0 0.66; 0 0 0.66; 2.1 0 0.66; 2.1 0 1.2; -0.9 0 1.2; -0.9 0 1.2; -0.9 0 1.2];
    angle = [0 0 0; 0 0 0; 0 0 0; 0 0 pi; 0 0 pi; 0 pi/3 pi; 0 0 pi];
    phx.Script(exc, {"Position", time, position, "pchip"}, {"EulerAngles", time, angle});

    % Camera following the bucket
    phx.Camera(ground, exc, "PointA", [-3, 6.6, 3]);

    % Simulation; contact tolerances for stones of 78 mm
    opt = phx.engine.Settings("Threads", 8, "LengthScale", 0.1);
    sim = phx.Simulation(ax, "EngineSettings", opt, "FrameSpacing", 0.1); % Initialize the simulation
    sim.step(25.6, 2560); % Run the simulation
    delete(sim); % Clean up the simulation object

end
