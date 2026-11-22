function phxex_dambreak
% PHXEX_DAMBREAK Water column collapsing in a tank and carrying a floating crate
%
% A column of water, a phx.Liquid that fills a box shape with particles,
% collapses into a long tank, runs along its floor and surges up the far
% wall. A light wooden crate set on top of the column falls with it, floats
% and is carried by the flow; a heavy concrete block in the middle of the
% tank stays on the floor, and the wave only pushes it a little. The front
% wall of the tank collides but is not drawn, so that the water can be
% seen.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-0.5 -1.6 1.0], "DefaultCameraTarget", [0 0 0.2]);

    % Tank of 1.2 x 0.4 m, its walls high enough to keep the surge in; the
    % friction holds the block (the water itself has none)
    tank = phx.assembly.arena(ax, "Size", [1.2 0.4 0.5], "Thickness", 0.04, "Friction", 0.6);
    set(tank.walls(3), "Visible", false);

    % Column of water 0.2 m wide and 0.4 m high against the left wall,
    % particles 1 cm apart; drawn as its surface
    phx.Liquid(ax, "Shape", {"Box", "Size", [0.2 0.4 0.4]}, "Position", [-0.5 0 0.2], ...
        "Spacing", 0.01, "SoundSpeed", 10, "Viscosity", 0.1);

    % Concrete block in the middle of the tank: heavier than water, it does
    % not float, and its friction on the floor lets the wave shift it only
    % a little
    phx.Body(ax, "Position", [0.1 0 0.1], "Friction", 0.4, ...
        "Shape", {"Box", "Size", [0.1 0.15 0.2], "Color", [0.6 0.6 0.58], "Texture", "stone", "TextureBlend", 0.3}, ...
        "Density", 2500);

    % Crate of light wood resting on top of the column
    phx.Body(ax, "Position", [-0.5 0 0.43], "Friction", 0.3, ...
        "Shape", {"Box", "Size", [0.1 0.1 0.06], "Color", [0.85 0.65 0.4], "Texture", "wood", "TextureBlend", 0.3}, ...
        "Density", 400);

    % Simulation; contact tolerances for parts of centimetres, steps of
    % 1 ms
    opt = phx.engine.Settings("LengthScale", 0.02, "Threads", 8);
    sim = phx.Simulation(ax, "EngineSettings", opt);
    sim.step(3, 3000);
    delete(sim);

end
