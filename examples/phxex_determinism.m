function phxex_determinism(numOfBalls)
% PHXEX_DETERMINISM Thousands of balls landing twice in the same spots
%
% Balls fall into an arena. Each is then coloured by the pixel of an image
% under the spot where it came to rest, all are reset to their start and
% the run is repeated: the simulation is deterministic, so they land in the
% same spots again and the image appears.
%
% phxex_determinism(n) uses n balls (default 2000).

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        numOfBalls (1, 1) double = 2000
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Figure setup
    rng(0); % Set random seed for reproducibility
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [50, 22, 89], "Texture", "Checker"); % Set camera position and view mode

    % Physical model: Create static ground bodies
    phx.assembly.arena("Size", [50 50 2], "Thickness", 1);

    % Create dynamic balls scattered in a tall column above the arena
    ballShape = phx.shape.Sphere("Diameter", 1.41, "Segments", 14);
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