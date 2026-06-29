function phxex_jenga(n, s)
% PHXEX_JENGA Simulates a Jenga-like stacking game
%
% Input Arguments:
%     n - number of blocks to stack (default: 15)
%     s - size of the blocks (default: [2 6 1])

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        n (1, 1) double = 15
        s (1, 3) double = [2 6 1]
    end

    % Figure setup
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 10], "Texture", "defaultNebula");

    % Physical model
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 1]});
    shpA = phx.shape.Box("Size", s, "Style", "edged", "ForcePatch", true);
    shpB = phx.shape.Box("Size", s([2 1 3]), "Style", "edged", "ForcePatch", true);
    clr = (jet(n) + 1)/2; % Generate colors for the blocks
    % Create layers with three blocks each
    for i = 1:n
        if mod(i, 2) == 1 % Alternate between two shapes for odd/even blocks
            shpA.Color = clr(i, :);
            sticks(i*3 - 2) = phx.Body(ax, "Position", [-s(1) rand i*s(3)], "Shape", shpA, "UserData", [0 1 0], "OnDoubleClickFcn", @moveOut);
            sticks(i*3 - 1) = phx.Body(ax, "Position", [0 rand i*s(3)], "Shape", shpA, "UserData", [0 1 0], "OnDoubleClickFcn", @moveOut);
            sticks(i*3 - 0) = phx.Body(ax, "Position", [s(1) rand i*s(3)], "Shape", shpA, "UserData", [0 1 0], "OnDoubleClickFcn", @moveOut);
        else
            shpB.Color = clr(i, :);
            sticks(i*3 - 2) = phx.Body(ax, "Position", [rand -s(1) i*s(3)], "Shape", shpB, "UserData", [1 0 0], "OnDoubleClickFcn", @moveOut);
            sticks(i*3 - 1) = phx.Body(ax, "Position", [rand 0 i*s(3)], "Shape", shpB, "UserData", [1 0 0], "OnDoubleClickFcn", @moveOut);
            sticks(i*3 - 0) = phx.Body(ax, "Position", [rand s(1) i*s(3)], "Shape", shpB, "UserData", [1 0 0], "OnDoubleClickFcn", @moveOut);
        end
    end
    simBodies = [ground sticks]; % Combine ground and sticks into a single array

    % UI
    uibutton(gcf, "Text", "↺", "Position", [50, 100, 50, 50], "FontSize", 24, "ButtonPushedFcn", @goBack);

    % Callbacks
    function moveOut(body, event)
        % Move the selected block out of the stack
        set(gca, "PickableParts", "none", "HitTest", "off");
        vec = body.UserData*(1 + rand)*sign(rand - 0.5)*0.2; % Random movement vector
        simBodies.storeState; % Store the current state of the simulation
        sim = phx.Simulation(simBodies); % Create a new simulation instance
        body.Type = "kinematic"; % Set the body type to kinematic for controlled movement

        % Physical animation
        viewer.displayText(["Pulling", ""]);
        for j = 1:100
            if j < 50
                body.Position = body.Position + vec; % Move the block
            elseif j == 50
                body.Type = "dynamic"; % Change to dynamic type for physics simulation
            end
            sim.step(0.1, 10); % Step the simulation
            prog = '----------'; % Progress indicator
            prog(1:round(sim.Time)) = '>'; % Update progress based on simulation time
            viewer.displayText(prog, "replacelast");
            pause(0);
        end

        viewer.displayText("");
        delete(sim); % Clean up simulation object
        set(gca, "PickableParts", "visible", "HitTest", "on"); % Restore pickable parts
    end

    function goBack(~, ~)
        % Restore the previous state of the simulation
        state = simBodies.restoreState;
        simBodies.clearStates(state);
        viewer.displayText("State reverted", "below");
    end

end