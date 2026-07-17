function phxex_stairfall(pushSpeed, duration)
% PHXEX_STAIRFALL Ragdoll on a staircase - a URDF humanoid tumbles downstairs
%
% A simple humanoid described in a standard URDF file (res/human.urdf:
% box pelvis, torso and feet, a sphere head and capsule limbs, connected
% by twelve passive revolute joints) is imported with phx.assembly.import,
% placed at the top of a staircase, leaned forward and given a small push.
%
% Every part of the fall is contact-driven - which step the shin catches
% on, how the shoulder slams into an edge, whether the body cartwheels or
% slides the last few steps - so the motion cannot be scripted or computed
% in closed form. It emerges from the interplay of thirteen rigid bodies,
% twelve joint constraints and dozens of transient contacts.
%
% Trace draws the trajectory of the head and a phx.Logger records its
% velocity, from which the demo reports the peak head speed and the
% largest impact deceleration of the ride.
%
% Input Arguments:
%     pushSpeed - initial forward speed of the whole body in m/s
%     duration  - maximum simulated time in seconds
%
% Example:
%     phxex_stairfall          % default gentle push
%     phxex_stairfall(2.5)     % running start

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        pushSpeed (1, 1) double = 1.2
        duration (1, 1) double {mustBePositive} = 6
    end

    % Staircase parameters (a common residential stair: 170 mm rise, 280 mm run)
    rise = 0.20;
    run = 0.28;
    nSteps = 8;
    width = 1.6;
    H = nSteps*rise;                   % height of the top landing

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [1.1 0 0.9], ...
        "DefaultCameraPosition", [4.6 -5.2 2.7]);

    % Top landing, the flight of steps and the floor at the bottom. The
    % steps are full-height static boxes so there are no gaps to fall into.
    phx.Body(ax, "Type", "static", "Position", [-0.7 0 H/2], ...
        "Shape", {"Box", "Size", [1.4 width H], "Color", [0.55 0.53 0.5]}, ...
        "Friction", [0.7 0 0]);
    for i = 1:nSteps - 1
        h = H - i*rise;
        phx.Body(ax, "Type", "static", "Position", [(i - 0.5)*run, 0, h/2], ...
            "Shape", {"Box", "Size", [run width h], ...
            "Color", [0.62 0.6 0.57] - 0.05*mod(i, 2)}, ...
            "Friction", [0.7 0 0]);
    end
    ground = phx.Body(ax, "Type", "static", "Position", [0, 0, -0.1], ...
        "Shape", {"Box", "Size", [(nSteps - 1)*run*4, (nSteps - 1)*run*4, 0.2], ...
        "Color", [0.75 0.73 0.7], "Texture", resdir+"tiles.jpg"}, "Friction", [0.7 0 0]);

    phx.Body(ax, "Position", [-0.2 0 H+0.2], "Shape", {"STL", "Source", resdir+"cat.stl", "Scale", 0.01, "Color", 1});

    % Import the humanoid right at the edge of the landing, leaning over it
    lean = 0.647; %randi(1000)/1000
    [bodies, joints] = phx.assembly.import(resdir + "human.urdf", ...
        "Position", [-0.16 0 H + 1.01], "EulerAngles", [0 lean 0]);

    % Bodies connected by a joint do not collide by default (the URDF
    % convention); let the head and feet knock against their counterparts.
    set([joints.neck joints.ankle_l joints.ankle_r], "MutualCollisions", true);

    names = fieldnames(bodies);
    human = phx.Body.empty;
    for i = 1:numel(names)
        human(i) = bodies.(names{i});   % pelvis first (the root link)
    end

    % Give the whole body a forward push
    for b = human
        b.LinearVelocity = [pushSpeed 0 0];
    end

    phx.Camera(ground, bodies.torso, "PointA", [4.6 -5.2 2.7]);

    % A mild velocity-proportional drag on every body part stands in for the
    % energy dissipation of soft tissue
    phx.Resistance(human, "VelocityFactors", [0 4]);

    % Trajectory of the head and its velocity record
    head = bodies.head;
    phx.Trace(head, "TracePoints", 600, "Color", [1 0.35 0.2], "Overlay", true);
    logVel = phx.Logger(head, "Frequency", 400, "Parameters", "LinearVelocity");
    logPos = phx.Logger(head, "Frequency", 400, "Parameters", "Position");

    % Simulation - dt = 2.5 ms for the joint chain and the step contacts
    sim = phx.Simulation(ax);
    frame = 0.02;
    subSteps = 8;
    calm = 0;
    while sim.Time < duration
        sim.step(frame, subSteps, 4);

        % Stop once the whole body has come to rest
        vmax = 0;
        for b = human
            vmax = max(vmax, norm(b.LinearVelocity));
        end
        if vmax < 0.25
            calm = calm + 1;
            if calm > 50      % a second of stillness
                break
            end
        else
            calm = 0;
        end

        viewer.displayText(sprintf("t = %4.2f s   head z = %.2f m", ...
            sim.Time, head.Position(3)));
        pause(0);
    end
    tEnd = sim.Time;
    pelvisEnd = bodies.pelvis.Position;
    delete(sim);

    % Ride statistics from the head velocity record
    t = logVel.Time;
    v = logVel.getChannel(1);
    speed = sqrt(sum(v.^2, 2));
    [vPeak, iPeak] = max(speed);
    decel = max(0, -diff(speed)./diff(t(:)));
    [aPeak, iDecel] = max(decel);
    fprintf("Tumble over after %.2f s; the pelvis ended %.2f m from the edge, %.2f m below the landing.\n", ...
        tEnd, pelvisEnd(1), H - pelvisEnd(3));
    fprintf("Peak head speed %.2f m/s at t = %.2f s; hardest head impact ~%.0f g at t = %.2f s.\n", ...
        vPeak, t(iPeak), aPeak/9.81, t(iDecel));

    % Head height and speed during the fall
    clf(figure(2));
    p = logPos.getChannel(1);
    yyaxis left
    plot(logPos.Time, p(:, 3), "LineWidth", 1.2);
    ylabel("head height [m]");
    yyaxis right
    plot(t, speed, "--", "LineWidth", 1.2);
    hold on
    plot(t(iDecel), speed(iDecel), "ro");
    ylabel("head speed [m/s]");
    grid on; xlabel("time [s]");
    title("The head on its way down the stairs");

end
