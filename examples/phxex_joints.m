function phxex_joints(showGraphs)
% PHXEX_JOINTS Revolute and spherical joints linking a chain of bodies
%
% Free bodies are linked to a static anchor by a phx.RevoluteJoint and a
% phx.SphericalJoint; phx.Logger records the velocity and joint forces.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true
    end

    % Clear the current figure, set 3D view, and configure lighting
    clf; view(3); axis("equal"); camlight("headlight"); grid("on");
    xlim([0 6]); ylim([-3 3]); zlim([-3 3]);

    % Create static body A
    A = phx.Body("Type", "static", "Position", [1 1 0], "Shape", {"Box", "SkeletPoints", [1.5 0 0]});

    % Create dynamic body B
    B = phx.Body("Position", [4 1 0], "Shape", {"Box", "SkeletPoints", [-1.5 0 0; 0 -1.5 0]});

    % Create dynamic body C
    C = phx.Body("Position", [4 -2 0], "Shape", {"Box", "SkeletPoints", [0 1.5 0]});

    % Create a revolute joint between bodies A and B
    phx.RevoluteJoint(A, B, "PointA", [1.5 0 0], "PointB", [-1.5 0 0], "Color", 0.5);

    % Create a spherical joint between bodies B and C
    S = phx.SphericalJoint(B, C, "PointA", [0 -1.5 0], "PointB", [0 1.5 0], "Color", 0.5);
    L2 = phx.Logger(S, "Frequency", 100, "Parameters", ["ForceA", "ForceB"]);

    % Logger to record the linear velocity of body B
    L = phx.Logger(B, "Parameters", "LinearVelocity", "Frequency", 100);

    % Simulation
    sim = phx.Simulation; % Create a simulation object
    sim.step(6, 600, 1);  % Run the simulation for 6 seconds with 600 steps
    delete(sim);          % Clean up the simulation object

    % Plot logged signal
    line(L.Time, L.Data, L.Time*0 - 3); % Plot the velocity data
    xlabel("time"); % Label for x-axis
    ylabel("box #2 velocity"); % Label for y-axis

    if showGraphs
        figure(2);
        plot(L2.Time, L2.getChannel(1) - 1e4, L2.Time, L2.getChannel(2) + 1e4);
    end

end