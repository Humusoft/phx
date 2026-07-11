function phxex_jenga_free(n, s)
% PHXEX_JENGA_FREE Simulates a free run of a Jenga-like structure
%
% Input Arguments:
%     n - number of blocks (default is 15)
%     s - size of the blocks in the format [width height depth] (default is [2 6 1])

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        n (1, 1) double = 15
        s (1, 3) double = [2 6 1]
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    [viewer, ax] = phx.extra.Viewer("newfigure", "DefaultCameraTarget", [0 0 10], "Texture", "defaultNebula");

    % Physical model: create a static base
    phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 1]});

    % Create shapes for the blocks
    shpA = phx.shape.Box("Size", s, "Texture", resdir+"woodtile.jpg", "TextureBlend", 0.5);
    shpB = phx.shape.Box("Size", s([2 1 3]), "Texture", resdir+"woodtile.jpg", "TextureBlend", 0.5);

    % Generate colors for the blocks
    clr = (jet(n) + 1)/2;

    % Loop to create and position layers of blocks
    for i = 1:n
        if mod(i, 2) == 1
            % Odd layers use shape A
            shpA.Color = clr(i, :);
            phx.Body(ax, "Position", [-s(1) rand i*s(3)], "Shape", shpA, "Friction", 1);
            phx.Body(ax, "Position", [0 rand i*s(3)], "Shape", shpA, "Friction", 1);
            phx.Body(ax, "Position", [s(1) rand i*s(3)], "Shape", shpA, "Friction", 1);
        else
            % Even layers use shape B
            shpB.Color = clr(i, :);
            phx.Body(ax, "Position", [rand -s(1) i*s(3)], "Shape", shpB, "Friction", 1);
            phx.Body(ax, "Position", [rand 0 i*s(3)], "Shape", shpB, "Friction", 1);
            phx.Body(ax, "Position", [rand s(1) i*s(3)], "Shape", shpB, "Friction", 1);
        end
    end

    % UI
    uialert(viewer.Figure, "Press F5 to run the simulation and then double-click on a brick to activate the editing mode.", "Jenga", "Icon", "info");

end