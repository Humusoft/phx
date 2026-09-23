function phxex_kinova(sweepTime, Tmax, smooth)
% PHXEX_KINOVA Imported Kinova Gen3 arm picking up a paddle and sweeping a tower with it
%
% The robot is not built here. phx.assembly.import turns the rigidBodyTree of
% loadrobot("kinovaGen3") into phx.Body parts and the phx.RevoluteJoint joints
% between them. The paddle is a loose body standing in a rack, belonging to
% nobody.
%
% The arm fetches it the way a machine changes a tool: it comes down onto the
% blade and couples to it with a phx.FixedJoint made while the scene is already
% running. Then it lifts the blade out of the rack, sweeps the tower off the
% table and lets the tool go. Robotics System Toolbox solves where the tool
% point should be, one inverseKinematics solution per waypoint, and every joint
% is servoed to its planned angle by its own motor. What that does to the tower
% is nobody's plan: the blocks are left to the contact solver.
%
% Whatever goes over the edge lands on a belt, a static box with a
% SurfaceVelocity, and is carried away by friction alone.
%
% Input Arguments:
%     sweepTime - seconds the paddle takes to cross the tower
%     Tmax      - torque ceiling of every joint motor in N*m
%     smooth    - ease each move in and out instead of running the joints at a
%                 constant rate between waypoints
%
% Example:
%     phxex_kinova            % a brisk sweep, the tower goes over the edge
%     phxex_kinova(4)         % slow push, the blocks are shoved aside
%     phxex_kinova(1, 4)      % weak motors, the tower stops the arm
%     phxex_kinova(1.2, 60, false)  % the arm jolts at every waypoint and rings
%
% See also phx.assembly.import, phx.Body/SurfaceVelocity, phx.RevoluteJoint

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        sweepTime (1, 1) double {mustBePositive} = 1.2
        Tmax (1, 1) double {mustBePositive} = 60
        smooth (1, 1) logical = true
    end

    % Table and tower
    tableAt = [0.45 0];             % table centre in the robot base frame
    tableTop = 0.20;
    blk = 0.03;                     % block edge
    nBlocks = 10;

    % Belt along the far side of the table, where the swept blocks come down
    belt = [0.55 1.1 0.02];
    beltSize = [1.60 0.52 0.04];    % length along the drive, width, thickness
    beltSpeed = 0.3;                % m/s, away from the robot

    % Tool rack and the paddle standing in it
    rackAt = [0.40 -0.38];
    rackTop = 0.10;
    paddle = [0.16 0.03 0.26];      % width across the sweep, thickness, length
    bladeTop = rackTop + paddle(3);
    grip = 0.02;                    % how close the tool point comes before it latches

    % Tool point waypoints and the time each is reached: over the rack, down
    % onto the blade, lift it out, across to the tower, through it, and away
    via = [rackAt  bladeTop + 0.12
           rackAt  bladeTop
           rackAt  bladeTop + 0.12
           tableAt(1) -0.30 tableTop + 0.03 + paddle(3)+0.1
           tableAt(1)  0.30 tableTop + 0.03 + paddle(3)+0.1
           rackAt(1) - 0.02, 0.38, tableTop + 0.03 + paddle(3)+0.1];
    hold = [0.4 1.2 1.6 2.4 2.4 + sweepTime, 3.2 + sweepTime];
    stage = ["waiting" "reaching for the paddle" "lifting it out" ...
             "carrying it over" "sweeping" "letting go"];

    % Joint servo
    Kp = 200;                       % rad/s per rad of error
    Vmax = 10;                      % rad/s

    % --- Plan the moves with Robotics System Toolbox ----------------------
    robot = loadrobot("kinovaGen3", "DataFormat", "row");
    ik = inverseKinematics("RigidBodyTree", robot);

    % Z of the tool point looks down and its X lies across the table, which is
    % how the blade ends up held, so the whole pose is asked for
    Q = zeros(size(via, 1), numel(homeConfiguration(robot)));
    q = [0 0.6 0 1.6 0 0.9 0];      % elbow bent over the table, not straight up
    for k = 1:size(via, 1)
        T = trvec2tform(via(k, :))*axang2tform([1 0 0 pi]);
        [q, info] = ik("EndEffector_Link", T, ones(1, 6), q);
        if info.PoseErrorNorm > 5e-3
            error("phx:kinova:unreachable", ...
                "Waypoint %d is out of the robot reach (pose error %.4f m).", k, info.PoseErrorNorm);
        end
        Q(k, :) = q;
    end

    % --- Scene -----------------------------------------------------------
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0.16 -0.41 0.42], "DefaultCameraPosition", [-0.66 -1.53 0.9]);

    ground = phx.Body(ax, "Type", "static", "Position", [0 0 -0.1], ...
        "Shape", {"Box", "Size", [4 4 0.2], "Color", [0.75 0.73 0.7], "Texture", "tiles"});
    phx.Body(ax, "Type", "static", "Position", [tableAt tableTop/2], ...
        "Shape", {"Box", "Size", [0.34 0.45 tableTop], "Color", [0.75 0.53 0.3], "Texture", "wood", "TextureBlend", 0.5});

    % Nothing about the belt moves, its SurfaceVelocity only makes contacts
    % solve as if the surface were sliding, along the local x the arrows show
    arrows = fullfile(fileparts(mfilename("fullpath")), "res", "arrows.png");
    phx.Body(ax, "Type", "static", "Position", belt, "EulerAngles", [0 0 pi/2], ...
        "Friction", [0.8 0 0], "SurfaceVelocity", [beltSpeed 0 0], ...
        "Shape", {"Box", "Size", beltSize, "Color", [0.8 0.2 0.6], ...
        "Texture", arrows, "TextureBlend", 0.5});

    % The rack: a stand and two guides that hold the blade upright, with a few
    % millimetres of clearance so it lifts out freely
    phx.Body(ax, "Type", "static", "Position", [rackAt rackTop/2], ...
        "Shape", {"Box", "Size", [0.24 0.18 rackTop], "Color", 0.3});
    for s = [-1 1]
        phx.Body(ax, "Type", "static", ...
            "Position", [rackAt(1), rackAt(2) + s*(paddle(2)/2 + 0.02), rackTop + 0.04], ...
            "Shape", {"Box", "Size", [0.18 0.025 0.08], "Color", 0.3});
    end
    blade = phx.Body(ax, "Position", [rackAt rackTop + paddle(3)/2], ...
        "Shape", {"Box", "Size", paddle, "Density", 300, "Color", [0.85 0.32 0.15]});

    % Import the arm in the pose the run starts from, then bolt it down. Its
    % base is held by a joint rather than made static, because a motor in a
    % joint that reaches a static body does not regulate, and the shoulder is
    % one. The tree gives the base no mass of its own either.
    [bodies, joints] = phx.assembly.import(ax, robot, "Configuration", Q(1, :));
    bodies.base_link.Mass = 20;
    bodies.base_link.Inertia = [0.2 0.2 0.2];
    phx.FixedJoint(ground, bodies.base_link, "PointA", [0 0 0.1], "PointB", [0 0 0], ...
        "Visible", false);

    motor = [joints.Actuator1 joints.Actuator2 joints.Actuator3 joints.Actuator4 ...
             joints.Actuator5 joints.Actuator6 joints.Actuator7];
    for j = motor
        j.MaxTorque = Tmax;
    end

    % The tool point sits beyond the last body of the tree, so it is carried as
    % a fixed offset in the frame of the wrist
    hand = bodies.Bracelet_Link;
    Ttool = hand.Transform\(trvec2tform(via(1, :))*axang2tform([1 0 0 pi]));

    % The tower in the way, tapering so the big blocks are at the bottom
    block = phx.Body.empty;
    for i = 1:nBlocks
        taper = (1 - i/nBlocks)*0.1;
        block(i) = phx.Body(ax, "Position", [tableAt tableTop + blk*(i - 0.5)], ...
            "Shape", {"Box", "Size", [blk+taper blk+taper blk], "Color", [0.35 0.5 0.8] + 0.08*mod(i, 2)});
    end

    % --- Run -------------------------------------------------------------
    % An imported joint reads the joint position of the tree in its Angle, so
    % the configurations the solver returned are servoed exactly as they came
    plan = struct("Time", [0 hold], "Q", [Q(1, :); Q], "Kp", Kp, "Vmax", Vmax, ...
        "Smooth", smooth);
    phx.Function(motor, @(o, p, ~, time) armStep(p, time, plan));

    % A 40 mm collision margin, the default, would inflate the links of a robot
    % this size into a permanent interpenetration and push its joints apart
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.BulletSettings("Margin", 0.002));

    % The coupler opens while the arm is already pulling away, so the blade
    % leaves with the motion it had and topples instead of landing on its end
    letGo = hold(end - 1) + 0.35;
    duration = plan.Time(end) + 2.6;
    picked = 0;
    coupler = phx.FixedJoint.empty;
    while sim.Time < duration
        sim.step(0.01, 10, 10);     % 1 ms substeps for the joint chain

        if picked == 0 && sim.Time > hold(2) - 0.3
            [picked, coupler] = latch(sim, hand, Ttool, blade, paddle(3), grip);
        elseif ~isempty(coupler) && sim.Time > letGo
            delete(coupler);        % from here the paddle is a free body again
            coupler = phx.FixedJoint.empty;
        end

        [~, seg] = poseAt(sim.Time, plan);
        viewer.displayText(sprintf("t = %4.2f s   tracking error %5.3f rad   %s", ...
            sim.Time, trackingError(motor, plan, sim.Time), stage(seg)));
    end
    delete(sim);

    % --- Result ----------------------------------------------------------
    if picked == 0
        warning("phx:kinova:noPickup", "The arm never reached the paddle, so it swept nothing.");
    end
    z = vertcat(block.Position)*[0; 0; 1];
    fprintf("Picked the paddle up at t = %.2f s, swept %d of %d blocks off the table " + ...
        "and held the plan to %.3f rad.\n", picked, sum(z < tableTop), nBlocks, ...
        trackingError(motor, plan, duration));

end

function [picked, coupler] = latch(sim, hand, Ttool, blade, bladeLen, grip)
% Couples the paddle to the wrist once the tool point reaches the top of the
% blade, and returns the time it happened at (0 while it has not).
    picked = 0;
    coupler = phx.FixedJoint.empty;
    T = hand.Transform*Ttool;
    top = blade.Transform*[0; 0; bladeLen/2; 1];
    if norm(T(1:3, 4) - top(1:3)) > grip
        return
    end

    % Welded where they stand, so the blade goes on hanging the way it stood
    coupler = phx.FixedJoint(hand, blade, "TransformA", hand.Transform\blade.Transform, ...
        "TransformB", eye(4), "Visible", false);

    % A joint made while the scene runs enters the pipelines only on a rebuild,
    % which addObjects asks for; the paddle being there already is all the
    % warning would be about
    w = warning("off", "phx:Object:duplicateParent");
    sim.addObjects(blade);
    warning(w);

    picked = sim.Time;
end

function armStep(joints, time, plan)
% Servos every joint to the angle the planned move asks for at this instant.
    q = poseAt(time, plan);
    for k = 1:numel(joints)
        j = joints{k};
        j.TargetVelocity = max(-plan.Vmax, min(plan.Vmax, ...
            plan.Kp*shortestWay(q(k) - j.Angle)));
    end
end

function e = trackingError(motor, plan, time)
% Largest angle by which a joint lags the plan, the price of the collision.
    e = max(abs(shortestWay(poseAt(time, plan) - [motor.Angle])));
end

function [q, seg] = poseAt(time, plan)
% Configuration the plan asks for at this instant, and which move it is in.
% Unsmoothed, a joint runs at a constant rate across a move, so the commanded
% rate steps at every waypoint and the arm rings on its own compliance.
% Smoothed, the rate ramps over the first and last quarter of the move. A
% quarter rather than the whole of it: easing all the way across has to peak at
% 1.9 times the average rate, which bends the joints more than the jolt did.
    t = min(time, plan.Time(end));
    seg = min(find(plan.Time <= t, 1, "last"), numel(plan.Time) - 1);
    s = (t - plan.Time(seg))/(plan.Time(seg + 1) - plan.Time(seg));

    if plan.Smooth
        r = 0.25;
        if s < r
            s = s*s/(2*r*(1 - r));
        elseif s > 1 - r
            s = 1 - (1 - s)^2/(2*r*(1 - r));
        else
            s = (s - r/2)/(1 - r);
        end
    end
    q = plan.Q(seg, :) + s*(plan.Q(seg + 1, :) - plan.Q(seg, :));
end

function e = shortestWay(e)
% Joint.Angle is wrapped to [-pi, pi] while the solver hands out angles beyond
% it, so the error is wrapped too, or a joint would chase a reading it can
% never show and just keep turning.
    e = mod(e + pi, 2*pi) - pi;
end
