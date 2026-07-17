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
    phx.assembly.arena("Size", [50 50 2], "Thickness", 1);

    % Create dynamic balls scattered in a tall column above the arena
    ballShape = phx.shape.Sphere("Diameter", 1.41, "Division", 2);
    balls = phx.assembly.scatter(ballShape, numOfBalls, "Region", [40 40 numOfBalls*0.1], ...
        "Spacing", 1.41, "Position", [0 0 10], "Color", hsv(numOfBalls));
    balls.storeState("initial_state"); % Store the initial state of the balls

    % Simulation I: Run the first simulation
    sim = phx.Simulation; % Create simulation with ground and balls
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
    sim = phx.Simulation; % Create a new simulation for the second run
    sim.step(10, 1000, 10); % Step the simulation
    delete(sim); % Clean up the simulation object

end