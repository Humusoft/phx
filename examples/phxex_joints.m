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
    xlim([-5 5]); ylim([-5 5]); zlim([-5 5]);

    % Create static body A
    A = phx.Body("Type", "static", "Shape", {"Box", "SkeletPoints", [1.5 0 0; -3 0 3]}, "Restitution", 0.5);

    % Create dynamic body B
    B = phx.Body("Position", [3 0 0], "Shape", {"Box", "SkeletPoints", [-1.5 0 0; 0 -1.5 0]});

    % Create dynamic body C
    C = phx.Body("Position", [3 -3 0], "Shape", {"Box", "SkeletPoints", [0 1.5 0]});

    % Create dynamic body D
    D = phx.Body("Position", [3 -3 -1.5], "Shape", {"Box", "SkeletPoints", [0 0 1.5]});

    % Create dynamic body E
    E = phx.Body("Position", [-3 0 3], "EulerAngles", [0 pi/4 0], "Shape", {"Box"}, "Restitution", 1);

    % Create a revolute joint between bodies A and B
    phx.RevoluteJoint(A, B, "PointA", [1.5 0 0], "PointB", [-1.5 0 0], "Color", 0.5);

    % Create a spherical joint between bodies B and C
    S = phx.SphericalJoint(B, C, "PointA", [0 -1.5 0], "PointB", [0 1.5 0], "Color", 0.5);
    L2 = phx.Logger(S, "Frequency", 100, "Parameters", ["ForceA", "ForceB"]);

    % Create a fixed joint between bodies C and D
    phx.FixedJoint(C, D, "PointA", [0 0 -1.5], "Visible", false);

    % Create a fixed joint between bodies A and E
    phx.PrismaticJoint(A, E, "EulerAnglesA", [0 pi/4 0], "MutualCollisions", true);

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