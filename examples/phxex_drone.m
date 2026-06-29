function phxex_drone(wpScale)
% PHXEX_DRONE Quadcopter waypoint flight with four phx.Thruster rotors
%
% A quadcopter built from the drone STL meshes carries four phx.Thruster
% actuators on its motor mounts. Each thruster pushes along the local Z
% axis of the frame and adds the propeller reaction torque, with a short
% first-order lag modeling the motor response. The spinning propellers
% are visual-only kinematic bodies that follow the frame.
%
% A cascade controller runs at the full simulation rate: a position PD
% loop converts the waypoint error into a desired acceleration, which
% defines the required total thrust and the desired tilt of the frame;
% a geometric attitude PD loop turns the tilt error into body torques;
% and an exact allocation matrix distributes [thrust, roll, pitch, yaw]
% into the four motor throttles - the only actuation the drone has. The
% frame is longitudinally asymmetric (the rear arms are longer), so the
% allocation also balances the unequal lever arms.
%
% The drone takes off from the pad, flies a rectangle of waypoints with
% altitude changes, returns and lands. A ball resting on a slim column
% stands in the way of the first leg: the drone knocks it off, the ball
% falls and rolls away, and the attitude controller recovers from the
% collision disturbance. The final plots show the 3D trajectories of
% both the drone and the ball, and the altitude and throttle histories.
%
% Input Arguments:
%     wpScale - size of the waypoint rectangle (default 3)
%
% Example:
%     phxex_drone        % default flight
%     phxex_drone(5)     % larger rectangle

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        wpScale (1, 1) double {mustBePositive} = 3
    end

    % Drone parameters (the STL is in mm, scaled 0.01 -> ~1.8 m span)
    scl = 0.01;
    mounts = [0.495 0.495 0.08; 0.495 -0.495 0.08; -0.495 0.495 0.08; -0.495 -0.495 0.08];
    spins = [1 -1 -1 1];   % propeller spin directions (diagonal pairs)
    mDrone = 3;            % mass (kg)
    iDrone = [0.3 0.3 0.5];% inertia (kg*m^2)
    maxThrust = 18;        % per motor (N), hover at ~40 %
    kQ = 0.02;             % reaction torque per unit thrust (N*m/N)
    grav = 9.81;

    % Controller gains
    KpP = 1.8; KdP = 2.4;          % position -> acceleration
    KattP = 20; KattD = 3.5;       % tilt -> body torque
    KyawP = 2; KyawD = 1.5;        % heading hold

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [wpScale/2 wpScale/2 1.2], ...
        "DefaultCameraPosition", [wpScale + 6, -5, 4]);

    % Ground and the landing pad
    phx.Body(ax, "Type", "static", "Position", [wpScale/2 wpScale/2 -0.5], ...
        "Shape", {"Box", "Size", [4*wpScale, 4*wpScale, 1], "Color", [1 1 1]});
    phx.Body(ax, "Type", "static", "Position", [0 0 0.05], ...
        "Shape", {"Box", "Size", [1.6 1.6 0.1], "Color", [0.2 0.8 0.9]});

    % A ball on a slim column right in the path of the first flight leg
    zBall = 2.0;
    phx.Body(ax, "Type", "static", "Position", [wpScale/2, -0.5, (zBall - 0.18)/2], ...
        "Shape", {"Cylinder", "Diameter", 0.15, "Height", zBall - 0.18, ...
        "Color", [0.6 0.6 0.65]});
    ball = phx.Body(ax, "Position", [wpScale/2, -0.5, zBall], ...
        "Shape", {"Sphere", "Diameter", 0.36, "Density", 20, ...
        "Color", [0.7 0.2 0.5]}, "Friction", 0.2);

    % The quadcopter frame; mass properties are set explicitly
    drone = phx.Body(ax, "Position", [0 0 0.36], ...
        "Shape", {resdir+"drone_frame.stl", "Scale", scl, "Color", [0.8 0.8 0.8], ...
        "Material", "matte", "Style", "flat"});
    drone.Mass = mDrone;
    drone.Inertia = iDrone;

    % Four thrusters on the motor mounts with alternating reaction torque
    for i = 1:4
        th(i) = phx.Thruster(drone, "Point", mounts(i, :), "Direction", [0 0 1], ...
            "MaxThrust", maxThrust, "TimeConstant", 0.04, ...
            "ReactionFactor", spins(i)*kQ, "ForceVectorSize", 0.04, ...
            "Color", [0.8 0 0]); %#ok<AGROW> four rotors
    end

    % Visual-only spinning propellers following the frame
    shpProp = phx.shape.STL("Source", resdir+"drone_prop.stl", "Scale", scl, ...
        "Color", [0.9 0.65 0.2], "Style", "flat");
    for i = 1:4
        props(i) = phx.Body(ax, "Type", "kinematic", "Collisions", false, ...
            "Position", [0 0 1], "Shape", shpProp); %#ok<AGROW> four propellers
    end
    spinAng = zeros(1, 4);

    phx.Trace(drone, "TracePoints", 1500, "Overlay", true, "Color", [0.3 0.3 0.3]);

    % Exact thrust allocation: [T tauX tauY tauZ]' = A*f -> f = A\...
    A = [ones(1, 4); mounts(:, 2)'; -mounts(:, 1)'; spins*kQ];
    Ainv = inv(A); %#ok<MINV> precomputed once, used every substep

    % Waypoints: take off, fly a rectangle with altitude changes, land
    wp = [0 0 2.2; wpScale 0 2.2; wpScale wpScale 1.2; 0 wpScale 2.2; 0 0 2.2; 0 0 0.45];

    viewer.displayText("Take-off...");

    dt = 0.005;
    subSteps = 10;
    tMax = 60;

    % Cascade flight controller as a pipeline element.
    % It reads the drone state and drives the four thruster throttles every
    % substep, keeping its waypoint/landing state in UserData. The run loop
    % below only advances the engine and updates the propeller visuals.
    prm.th = th;  prm.wp = wp;  prm.Ainv = Ainv;
    prm.mDrone = mDrone;  prm.grav = grav;  prm.maxThrust = maxThrust;
    prm.KpP = KpP;  prm.KdP = KdP;
    prm.KattP = KattP;  prm.KattD = KattD;  prm.KyawP = KyawP;  prm.KyawD = KyawD;
    ctrl = phx.Function(drone, @(o, p, ~, tm) droneControl(o, p, tm, prm));
    ctrl.UserData = struct("wpIdx", 1, "landed", false, "tLand", NaN);

    sim = phx.Simulation(ax, "EngineSettings", ...
        phx.engine.BulletSettings("AutoActivated", false));

    log = struct("t", [], "p", [], "wp", [], "f", [], "pb", []);
    while sim.Time < tMax && (isnan(ctrl.UserData.tLand) || sim.Time < ctrl.UserData.tLand + 1.5)
        sim.step(dt*subSteps, subSteps, 1);   % controller runs every substep inside

        % Spin and place the visual propellers
        M = drone.Transform;
        for i = 1:4
            spinAng(i) = spinAng(i) + spins(i)*(8 + 50*th(i).Throttle)*dt*subSteps;
            c = cos(spinAng(i)); sn = sin(spinAng(i));
            L = [c -sn 0 mounts(i, 1); sn c 0 mounts(i, 2); 0 0 1 mounts(i, 3) + 0.06; 0 0 0 1];
            props(i).Transform = M*L;
        end
        pause(0);

        log.t(end + 1) = sim.Time;
        log.p(:, end + 1) = drone.Position;
        log.wp(end + 1) = ctrl.UserData.wpIdx;
        log.f(:, end + 1) = [th.Throttle];
        log.pb(:, end + 1) = ball.Position;
        viewer.displayText(sprintf("Waypoint %d / %d   alt %.2f m   throttle %3.0f %%", ...
            ctrl.UserData.wpIdx, size(wp, 1), drone.Position(3), 100*mean([th.Throttle])));
    end
    landed = ctrl.UserData.landed;
    tLand  = ctrl.UserData.tLand;
    wpIdx  = ctrl.UserData.wpIdx;
    delete(sim);

    pEnd = drone.Position;
    if landed
        fprintf("Landed %.2f m from the pad center after %.1f s of flight.\n", ...
            norm(pEnd(1:2)), tLand);
    else
        fprintf("Flight timed out at waypoint %d.\n", wpIdx);
    end
    pb = ball.Position;
    fprintf("The ball was knocked %.2f m away from its column.\n", ...
        norm(pb(1:2) - [wpScale/2, 0]));
    viewer.displayText(sprintf("Landed:  offset %.2f m,  flight time %.1f s", norm(pEnd(1:2)), tLand));

    % Trajectory and flight history plots
    figure(2);
    subplot(1, 2, 1);
    plot3(log.p(1, :), log.p(2, :), log.p(3, :), "LineWidth", 1.5); hold on
    plot3(wp(:, 1), wp(:, 2), wp(:, 3), "o--", "LineWidth", 1);
    plot3(log.pb(1, :), log.pb(2, :), log.pb(3, :), "LineWidth", 1.5);
    grid on; axis equal; view(40, 25);
    xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");
    legend("drone", "waypoints", "ball", "Location", "northeast");
    title("Waypoint flight");
    subplot(1, 2, 2);
    yyaxis left
    plot(log.t, log.p(3, :), "LineWidth", 1.5); ylabel("altitude [m]");
    yyaxis right
    plot(log.t, 100*mean(log.f, 1), "LineWidth", 1); ylabel("mean throttle [%]");
    xline(log.t([true diff(log.wp) > 0]), ":");
    grid on; xlabel("time [s]");
    title("Altitude and throttle");

end

function droneControl(o, parents, time, prm)
% Cascade flight controller run each substep by the phx.Function: a position
% PD loop -> desired acceleration and thrust vector, a geometric attitude PD
% loop -> body torques, and an exact allocation into the four motor thrusts.
% Waypoint and landing state are kept in the phx.Function UserData.
    drone = parents{1};
    s = o.UserData;     % .wpIdx .landed .tLand

    M = drone.Transform;
    R = M(1:3, 1:3);
    p = drone.Position;
    v = drone.LinearVelocity;
    w = drone.AngularVelocity;

    % Waypoint switching; the last waypoint is the landing descent
    e = prm.wp(s.wpIdx, :) - p;
    if norm(e) < 0.35 && norm(v) < 0.6 && s.wpIdx < size(prm.wp, 1)
        s.wpIdx = s.wpIdx + 1;
    end
    if s.wpIdx == size(prm.wp, 1) && p(3) < 0.5 && ~s.landed
        s.landed = true;
        s.tLand = time;
    end

    if s.landed
        f = zeros(4, 1);    % motors off after touchdown
    else
        % Position loop: desired acceleration and thrust vector
        aDes = prm.KpP*e - prm.KdP*v;
        aDes(1:2) = max(min(aDes(1:2), 3.5), -3.5);
        aDes(3) = max(min(aDes(3), 6), -5);
        Fvec = prm.mDrone*(aDes + [0 0 prm.grav]);

        % Attitude loop: align the body Z axis with the thrust vector
        nDes = Fvec/norm(Fvec);
        nCur = R(:, 3)';
        eAtt = cross(nCur, nDes);
        psi = atan2(R(2, 1), R(1, 1));
        tauW = prm.KattP*eAtt - prm.KattD*[w(1) w(2) 0] + [0 0 -prm.KyawP*psi - prm.KyawD*w(3)];
        tauB = R'*tauW';

        % Allocation into the four motor thrusts
        f = prm.Ainv*[max(Fvec*nCur', 0.1*prm.mDrone*prm.grav); tauB];
    end
    for i = 1:4
        prm.th(i).Throttle = max(min(f(i)/prm.maxThrust, 1), 0);
    end

    o.UserData = s;
end
