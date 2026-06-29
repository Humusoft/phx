function phxex_gravity(simTime, particleCount, showGraphs)
% PHXEX_GRAVITY Simulates the motion of particles under gravity
%
% Input Arguments:
%     simTime - total simulation time (default is 1000)
%     particleCount - total count of particles (default is 200)

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        simTime (1, 1) double = 1000
        particleCount (1, 1) double = 128
        showGraphs (1, 1) logical = true
    end

    % Set parameters for the simulation
    particleDensity = 1e8;
    particleSize = 1;
    rng(7); % world number

    % Clear the current figure and set up the viewer
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "ViewMode", "plain", "DefaultCameraPosition", [0 -100 0], "ViewMode", "plain");

    % Create a particle shape for the simulation
    particleShape = phx.shape.Sphere("Diameter", particleSize, "Division", 1, "Density", particleDensity);

    % Initialize particles with random positions and velocities
    for i = 1:particleCount
        theta = rand*2*pi;
        r = 15 + rand*10;
        d = (rand - 0.5)*5;
        particleShape = particleShape.nextColor;
        planets(i) = phx.Body(ax, "Position", [cos(theta)*r d sin(theta)*r], ...
                               "LinearVelocity", [-sin(theta) 0 cos(theta)]*sqrt(r)*1e-1, ...
                               "Shape", particleShape);
        % phx.Trace(planets(i), "Color", particleShape.Color, "TracePoints", 10);
    end

    % Create a central body (star) for the simulation
    centralShape = phx.shape.Sphere("Diameter", 1, "Division", 3, "Density", 1e11, "Color", [1 1 1]);
    planets(end+1) = phx.Body(ax, "Position", [0 0 0], "Shape", centralShape);

    % Set up logging for the central body's linear velocity
    log1 = phx.Logger(planets(end), "Parameters", "LinearVelocity", "Frequency", 10);
    phx.Trace(planets(end), "Color", centralShape.Color, "TracePoints", 1000);

    % Add gravity interaction to all bodies
    phx.Monopole(planets, "Attractivity", 6.67430e-11, "Charge", [planets.Mass], "Visible", false);

    % Create and run the simulation
    sim = phx.Simulation(planets, "Gravity", 0);
    sim.step(simTime, simTime*10, 10); % Step through the simulation
    delete(sim);

    % View logged data
    if showGraphs
        figure(2);
        plot(log1.Time, log1.Data); % Plot the velocity of the central star
        title("Central star velocity");
    end

end