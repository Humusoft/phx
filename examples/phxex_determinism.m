function phxex_determinism(numOfBalls)
% PHXEX_DETERMINISM A hardcore test of determinism
%
% This function sets up a physical simulation with thousands of dynamic
% balls.
%
% The simulation is run twice with the same initial conditions. The result
% of both runs is the same, as demonstrated by coloring the balls according
% to the image in the first run, which then composes the same image in the
% second run.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        numOfBalls (1, 1) double = 2000
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    rng(0); % Set random seed for reproducibility
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [50, 22, 89], "Texture", "defaultChecker"); % Set camera position and view mode

    % Physical model: Create static ground bodies
    ground(1) = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [50 50 1], "Color", [1 1 1]});
    ground(2) = phx.Body(ax, "Type", "static", "Position", [25.5 0 1], "Shape", {"Box", "Size", [1 50 2]});
    ground(3) = phx.Body(ax, "Type", "static", "Position", [-25.5 0 1], "Shape", {"Box", "Size", [1 50 2]});
    ground(4) = phx.Body(ax, "Type", "static", "Position", [0 25.5 1], "Shape", {"Box", "Size", [52 1 2]});
    ground(5) = phx.Body(ax, "Type", "static", "Position", [0 -25.5 1], "Shape", {"Box", "Size", [52 1 2]});

    % Create dynamic balls with random positions
    shp = phx.shape.Sphere("Diameter", 1.41, "Division", 2);
    for i = 1:numOfBalls
        balls(i) = phx.Body(ax, "Position", [rand*40-20 rand*40-20 10+i*0.1], "Shape", shp.nextColor);
    end
    balls.storeState("initial_state"); % Store the initial state of the balls

    % Simulation I: Run the first simulation
    sim = phx.Simulation([ground balls]); % Create simulation with ground and balls
    sim.step(10, 1000, 10); % Step the simulation
    delete(sim); % Clean up the simulation object

    % Read image for coloring the balls
    img = imread(resdir+"Newton.png");
    for i = 1:numOfBalls
        p = balls(i).Position*2; % Scale position for image indexing
        x = clip(round(p(1) + 50), 1, 100); % Clip x position to image bounds
        y = clip(round(p(2) + 50), 1, 100); % Clip y position to image bounds
        c = img(x, y, :); % Get color from image
        balls(i).Color = double(c)/255; % Set ball color
    end
    drawnow; % Update the figure window
    balls.restoreState("initial_state"); % Restore the initial state of the balls

    % Simulation II: Run the second simulation
    sim = phx.Simulation([ground balls]); % Create a new simulation for the second run
    sim.step(10, 1000, 10); % Step the simulation
    delete(sim); % Clean up the simulation object

end