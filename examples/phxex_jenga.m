function phxex_jenga(n, s)
% PHXEX_JENGA Simulates an interactive Jenga-like stacking game
%
% Double-click the mouse to pull out the brick from the stacked tower.
% You can undo the move and the tower's status using the button in the
% lower left corner of the window.
%
% Input Arguments:
%     n - number of blocks to stack (default: 15)
%     s - size of the blocks in metres (default: [0.2 0.6 0.1])

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        n (1, 1) double = 15
        s (1, 3) double = [0.2 0.6 0.1]
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Figure setup
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.8], "DefaultCameraPosition", [2.2 -3 1.7], ...
        "Texture", resdir+"HumusoftHall.png", "Lighting", "studio");

    % Physical model
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [5 5 0.1], "Color", [1 1 1]});
    shpA = phx.shape.Box("Size", s, "Texture", "wood", "TextureBlend", 0.4);
    shpB = phx.shape.Box("Size", s([2 1 3]), "Texture", "wood", "TextureBlend", 0.4);
    clr = (jet(n) + 1)/2; % Generate colors for the blocks
    mu = 0.04; % Friction of the blocks, low enough for a pulled block not to drag the tower along
    % Create layers with three blocks each
    for i = 1:n
        if mod(i, 2) == 1 % Alternate between two shapes for odd/even blocks
            shpA.Color = clr(i, :);
            sticks(i*3 - 2) = phx.Body(ax, "Position", [-s(1) 0.1*rand i*s(3)], "Shape", shpA, "UserData", [0 1 0], "OnDoubleClickFcn", @moveOut, "Friction", mu);
            sticks(i*3 - 1) = phx.Body(ax, "Position", [0 0.1*rand i*s(3)], "Shape", shpA, "UserData", [0 1 0], "OnDoubleClickFcn", @moveOut, "Friction", mu);
            sticks(i*3 - 0) = phx.Body(ax, "Position", [s(1) 0.1*rand i*s(3)], "Shape", shpA, "UserData", [0 1 0], "OnDoubleClickFcn", @moveOut, "Friction", mu);
        else
            shpB.Color = clr(i, :);
            sticks(i*3 - 2) = phx.Body(ax, "Position", [0.1*rand -s(1) i*s(3)], "Shape", shpB, "UserData", [1 0 0], "OnDoubleClickFcn", @moveOut, "Friction", mu);
            sticks(i*3 - 1) = phx.Body(ax, "Position", [0.1*rand 0 i*s(3)], "Shape", shpB, "UserData", [1 0 0], "OnDoubleClickFcn", @moveOut, "Friction", mu);
            sticks(i*3 - 0) = phx.Body(ax, "Position", [0.1*rand s(1) i*s(3)], "Shape", shpB, "UserData", [1 0 0], "OnDoubleClickFcn", @moveOut, "Friction", mu);
        end
    end
    simBodies = [ground sticks]; % Combine ground and sticks into a single array

    % UI
    uibutton(gcf, "Text", "↺", "Position", [50, 100, 50, 50], "FontSize", 24, "ButtonPushedFcn", @goBack);
    uialert(gcf, "Double-click on a brick to pull it out.", "Jenga", "Icon", "info");

    % Callbacks
    function moveOut(body, event)
        set(gca, "PickableParts", "none", "HitTest", "off");
        viewer.displayText(["Pulling", ""]);

        % Move the selected block out of the stack
        vel = body.UserData*(1 + rand)*sign(rand - 0.5)*0.7; % Random pulling velocity
        simBodies.storeState; % Store the current state of the simulation
        sim = phx.Simulation(simBodies, "PacingRate", 1); % Create a new simulation instance
        body.Type = "kinematic"; % Set the body type to kinematic for controlled movement
        body.LinearVelocity = vel; % Pull the block, it keeps this velocity
        sim.step(1, 100);
        body.LinearVelocity = [0 0 0]; % Stop pulling
        body.Type = "dynamic"; % Change to dynamic type for physics simulation
        sim.step(1, 100);
        delete(sim); % Clean up simulation object

        viewer.displayText("");
        set(gca, "PickableParts", "visible", "HitTest", "on"); % Restore pickable parts
    end

    function goBack(~, ~)
        % Restore the previous state of the simulation
        state = simBodies.restoreState;
        simBodies.clearStates(state);
        viewer.displayText("State reverted", "below");
    end

end