function phxex_stairfall(pushSpeed, duration)
% PHXEX_STAIRFALL Ragdoll on a staircase - a URDF humanoid tumbles downstairs
%
% A simple humanoid described in a standard URDF file (res/human.urdf:
% box pelvis, torso and feet, a sphere head and capsule limbs, connected
% by twelve passive revolute joints) is imported with phx.assembly.import,
% placed at the top of a staircase and given a small push at the torso.
%
% Every part of the fall is contact-driven - which step the shin catches
% on, how the shoulder slams into an edge, whether the body cartwheels or
% slides the last few steps - so the motion cannot be scripted or computed
% in closed form. It emerges from the interplay of the humanoid's thirteen
% rigid bodies, twelve joint constraints and dozens of transient contacts.
%
% Trace draws the trajectory of the head and a phx.Logger records its
% height and velocity, plotted after the run with the hardest impact of
% the ride marked.
%
% Input Arguments:
%     pushSpeed - initial forward speed of the torso in m/s
%     duration  - simulated time in seconds
%
% Example:
%     phxex_stairfall          % default gentle push
%     phxex_stairfall(2.5)     % running start

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        pushSpeed (1, 1) double = 1.5
        duration (1, 1) double {mustBePositive} = 6
    end

    % Staircase parameters
    rise = 0.20;
    run = 0.28;
    nSteps = 8;
    width = 1.6;
    H = nSteps*rise;                   % height of the top landing

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [1.1 0 0.9], ...
        "DefaultCameraPosition", [4.6 -5.2 2.7]);

    % Top landing, the flight of steps and the floor at the bottom. The
    % steps are full-height static boxes so there are no gaps to fall into.
    phx.Body(ax, "Type", "static", "Position", [-0.7 0 H/2], ...
        "Shape", {"Box", "Size", [1.4 width H], "Color", [0.55 0.53 0.5]}, ...
        "Friction", 0.7);
    for i = 1:nSteps - 1
        h = H - i*rise;
        phx.Body(ax, "Type", "static", "Position", [(i - 0.5)*run, 0, h/2], ...
            "Shape", {"Box", "Size", [run width h], ...
            "Color", [0.62 0.6 0.57] - 0.05*mod(i, 2)}, ...
            "Friction", 0.7);
    end
    ground = phx.Body(ax, "Type", "static", "Position", [0, 0, -0.1], ...
        "Shape", {"Box", "Size", [(nSteps - 1)*run*4, (nSteps - 1)*run*4, 0.2], ...
        "Color", [0.75 0.73 0.7], "Texture", "tiles"}, "Friction", 0.7);

    % A cat on the landing, at the humanoid's feet
    phx.Body(ax, "Position", [-0.2 0 H+0.2], "Shape", {"Mesh", "Source", resdir+"cat.stl", "Scale", 0.01, "Envelope", "box", "Color", 1});

    % Import the humanoid upright at the edge of the landing and push its
    % torso over the edge
    lean = 0.0;
    bodies = phx.assembly.import(resdir + "human.urdf", "Position", [-0.2 0 H + 1.01], "EulerAngles", [0 lean 0]);
    bodies.torso.LinearVelocity = [pushSpeed 0 0];

    phx.Camera(ground, bodies.torso, "PointA", [4.6 -5.2 2.7], "TrackingLag", 0.2);

    % Trajectory of the head and its velocity record
    head = bodies.head;
    phx.Trace(head, "TracePoints", 600, "Color", [1 0.35 0.2], "Overlay", true);
    logHead = phx.Logger(head, "Frequency", 400, "Parameters", ["LinearVelocity" "Position"]);

    % Simulation - 10 ms steps, paced to real time
    sim = phx.Simulation(ax, "PacingRate", 1);
    sim.step(duration, duration/0.01);
    delete(sim);

    % The hardest head impact from the velocity record
    t = logHead.Time;
    speed = vecnorm(logHead.getChannel(1), 2, 2);
    decel = max(0, -diff(speed)./diff(t(:)));
    [~, iDecel] = max(decel);

    % Head height and speed during the fall
    clf(figure(2));
    p = logHead.getChannel(2);
    yyaxis left
    plot(t, p(:, 3), "LineWidth", 1.2);
    ylabel("head height [m]");
    yyaxis right
    plot(t, speed, "--", "LineWidth", 1.2);
    hold on
    plot(t(iDecel), speed(iDecel), "ro");
    ylabel("head speed [m/s]");
    grid on; xlabel("time [s]");
    title("The head on its way down the stairs");

end
