function phxex_walker(freq, duty)
% PHXEX_WALKER Quadruped crawling on eight joint motors with no feedback
%
% Each leg is a thigh and a shank on two revolute joints, so the robot is
% driven by eight motorized joints and nothing else. A single phx.Function
% plays a prescribed crawl: the foot of every leg is asked to trace the
% same closed path - a straight line backwards while it carries the body,
% a lifted arc forwards while it steps - and the hip and knee angle that
% put it there come from the inverse kinematics of the two-link leg. The
% four legs run that path a quarter cycle apart, so three feet are always
% on the ground.
%
% Nothing in the loop ever looks at the body: the walk is entirely
% feedforward, only the joints themselves are servoed. A loose plank with
% a ball on it lies in the way to show what that costs - the crawl keeps
% playing unchanged while the robot pitches up onto the plank and over it,
% and it gets across on the stability of its stance alone.
%
% Input Arguments:
%     freq - gait frequency in Hz
%     duty - fraction of the cycle each leg spends on the ground
%
% Example:
%     phxex_walker            % 0.3 m per step on the flat
%     phxex_walker(1.5)       % faster cadence
%     phxex_walker(1, 0.6)    % two feet down at a time, the step collapses

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        freq (1, 1) double = 1.0
        duty (1, 1) double = 0.8
    end

    % Robot
    L = 0.20;                   % thigh and shank length
    Rl = 0.020;                 % leg radius, also the sole radius
    H = 0.22;                   % hip height, a crouch at 55% of the reach
    bx = [0.60 0.40 0.08];      % chassis size
    lx = 0.25;                  % hip offset along the body
    ly = 0.20;                  % half track
    Tmax = 5;                   % motor torque, N*m, near the robot weight so
                                % that a leg yields instead of fighting the others
    hip = [lx ly; lx -ly; -lx ly; -lx -ly];     % legs FL FR RL RR

    % Gait, all lengths in the frame of the hip the leg belongs to
    gait = struct( ...
        "L", L, ...
        "Height", H - Rl, ...       % foot depth in stance, sole on the floor
        "Stride", 1.4*L, ...        % stance stroke, so 0.28 m per cycle
        "Lift", 0.08, ...           % how high the foot clears in swing
        "Duty", duty, ...
        "Freq", freq, ...
        "Phase", [0 0.5 0.75 0.25], ... % crawl sequence LF-RH-RF-LH
        "Kp", 40, ...               % joint servo gain, rad/s per rad
        "Vmax", 25);                % joint servo velocity limit, rad/s

    duration = 20;
    dt = 0.02;

    % --- Scene ----------------------------------------------------------
    figure(1);
    viewer = phx.extra.Viewer("clear", "Texture", "sky");

    ground = phx.Body("Type", "static", "Position", [5 0 -0.1], ...
        "Shape", {"Box", "Size", [12 12 0.2], "Color", 0.8, "Texture", "tiles"}, ...
        "Friction", [10 0 0]);
    chassis = phx.Body("Position", [0 0 H+bx(3)/2], "Mass", 4, ...
        "Shape", {"Box", "Size", bx, "Color", [0.30 0.45 0.70]});

    % Obstacles
    phx.Body("Position", [2 0 0.1], "Shape", {"Box", "Size", [1 1 0.1], "Color", [1 1 0.5], "Texture", "wood", "TextureBlend", 0.3});
    phx.Body("Position", [2 0 0.4], "Shape", {"Globe", "Diameter", 0.5, "Color", [1 0 0], "Texture", "checker", "TextureBlend", 0.3});

    % Every leg starts in the pose its own gait phase asks for
    for k = 1:4
        [qh, qk] = legIK(footPath(gait.Phase(k), gait), L);
        hp = [hip(k, :) H];
        d1 = [-sin(qh) 0 -cos(qh)];
        d2 = [-sin(qh+qk) 0 -cos(qh+qk)];

        thigh(k) = phx.Body("Position", hp + (L/2)*d1, "EulerAngles", [0 qh 0], ...
            "Mass", 0.25, "Shape", {"Capsule", "Radius", Rl, "Height", L, "Color", [0.75 0.75 0.78]});
        shank(k) = phx.Body("Position", hp + L*d1 + (L/2)*d2, "EulerAngles", [0 qh+qk 0], ...
            "Mass", 0.2, "Friction", [10 0 0], ...
            "Shape", {"Capsule", "Radius", Rl, "Height", L, "Color", [0.55 0.55 0.60]});

        hipj(k) = phx.RevoluteJoint(chassis, thigh(k), ...
            "PointA", [hip(k, :) -bx(3)/2], "PointB", [0 0 L/2], ...
            "AxisA", [0 1 0], "AxisB", [0 1 0], "MaxTorque", Tmax, "Visible", false);
        kneej(k) = phx.RevoluteJoint(thigh(k), shank(k), ...
            "PointA", [0 0 -L/2], "PointB", [0 0 L/2], ...
            "AxisA", [0 1 0], "AxisB", [0 1 0], "MaxTorque", Tmax, "Visible", false);
    end

    phx.Function([hipj kneej], @(o, p, ~, time) walkStep(p, time, gait));
    phx.Camera(chassis, chassis, "PointA", [-1.0 1.0 1.0], "PointB", [1.5 0 0], ...
        "TrackingLag", 0.2);

    % --- Run ------------------------------------------------------------
    sim = phx.Simulation;
    x0 = chassis.Position(1);
    for t = dt:dt:duration
        sim.step(dt, 20, 20);       % 1 ms substeps, one redraw per frame
        travel = chassis.Position(1) - x0;
        viewer.displayText(sprintf("%4.1f s   %+.2f m   %+.2f m/s", ...
            sim.Time, travel, travel/sim.Time));
    end
    delete(sim);

end

function walkStep(joints, time, gait)
% Holds every joint at the angle its foot path asks for at this instant.
    for i = 1:4
        [qh, qk] = legIK(footPath(time*gait.Freq + gait.Phase(i), gait), gait.L);
        hj = joints{i};
        kj = joints{4+i};
        hj.TargetVelocity = max(-gait.Vmax, min(gait.Vmax, gait.Kp*(qh - hj.Angle)));
        kj.TargetVelocity = max(-gait.Vmax, min(gait.Vmax, gait.Kp*(qk - kj.Angle)));
    end
end

function p = footPath(phi, gait)
% Foot position [x z] in the frame of its hip at the gait phase phi.
    phi = mod(phi, 1);
    if phi < gait.Duty          % stance: a straight line carrying the body
        s = phi/gait.Duty;
        p = [gait.Stride*(0.5-s), -gait.Height];
    else                        % swing: a lifted arc stepping forwards
        s = (phi-gait.Duty)/(1-gait.Duty);
        p = [gait.Stride*(s-0.5), -gait.Height + gait.Lift*sin(pi*s)];
    end
end

function [qh, qk] = legIK(p, L)
% Hip and knee angle putting the foot of a two-link leg of equal links at p.
    u = -p(1);
    w = -p(2);
    d = min(hypot(u, w), 1.98*L);           % keep clear of the stretched leg
    qk = acos(max(-1, min(1, (d*d - 2*L*L)/(2*L*L))));
    qh = atan2(u, w) - atan2(L*sin(qk), L + L*cos(qk));
end
