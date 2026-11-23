function phxex_balancewater(Kp, Kd)
% PHXEX_BALANCEWATER Half-filled water ball rolled from stop to stop by a platform tilted by two servos
%
% A variation of phxex_balance: the ball is a hollow sphere twice the size,
% filled about halfway with water, and the plate is not moved directly. It
% sits on a column on a joint that lets it tilt but not turn, and two servo
% motors on the base tilt it through cranks and push rods (see
% buildPlatform below). The controller drives the ball from one random
% stop on the plate to the next, accelerating hard and braking hard, and
% holds it there for a while. The water does not roll with the shell - it
% slides inside, and every start and stop sets it sloshing, shifting the
% weight the controller and the motors have to fight. The PD law of
% phxex_balance plus the acceleration of the path asks for a plate tilt;
% inverse kinematics turns that tilt into crank angles, which the motors
% follow. The tilt follows smoothly: a jump would slam the heavy ball.
%
% Input Arguments:
%     Kp - proportional gain on ball position error
%     Kd - derivative gain on ball velocity error
%
% Example:
%     phxex_balancewater              % default gains
%     phxex_balancewater(0.6, 0.4)    % softer gains
%
% See also phxex_balance, phx.SphericalJoint, phx.RevoluteJoint, phx.Function

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Kp (1, 1) double = 8
        Kd (1, 1) double = 2
    end

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 -0.4], "DefaultCameraPosition", [5 -6 1.2], "Texture", "hall");

    % Plate geometry
    plateSize = [4 4 0.2];
    ballR = 0.6;
    wall = 0.03;
    maxTilt = 0.25;            % largest plate tilt the controller asks for [rad]

    % The plate on its column, the servo drives with cranks and push rods
    rig = buildPlatform(ax, plateSize, maxTilt);

    % Hollow ball resting off-centre on the plate, its wall half see-through;
    % a checkerboard of 8 x 4 squares shows it rolling
    p0 = [1.2 1.2 plateSize(3)/2 + ballR];
    ball = phx.Body(ax, "Position", p0, "Shape", {"SphereShell", "Radius", ballR, "WallThickness", wall, ...
        "Color", [1 0.5 0.2], "Material", "glossy", "Texture", "checker", ...
        "TextureBlend", 0.5, "Alpha", 0.4});

    % Its path: random stops, from where it rests
    path = randomStops(p0(1:2), 8, 1.2, 2.5);

    % A ball of water of half the cavity's volume, which settles to the
    % bottom; particles 4 cm apart
    d = 0.04;
    water = phx.Liquid(ax, "Position", p0, "Spacing", d, "SoundSpeed", 10, "Viscosity", 0.1, "Shape", {"Sphere", "Radius", ballR*0.75});

    % Visual trail of the ball path on the plate
    phx.Trace(ball, "TracePoints", 400, "Color", [1 0.7 0.3]);

    % Log the ball position and the motor torques
    logBall  = phx.Logger(ball, "Frequency", 50, "Parameters", "Position");
    logDrive = phx.Logger(rig.drive, "Frequency", 50, "Parameters", "TorqueA");

    % Acceleration of the ball per unit tilt (sine of the slope): the shell
    % rolls, the water slides inside without turning
    M = ball.Mass + water.Mass;
    kRoll = 9.81*M/(M + ball.Inertia(1)/ballR^2);

    % Closed-loop control: the PD control law and the servos run every
    % step as a phx.Function bound to the ball (sensor) and the drives;
    % UserData holds the commanded tilt and its rate
    ctrl = phx.Function({ball, rig.drive(1), rig.drive(2)}, ...
        @(fcn, ~, dt, t) balanceLaw(fcn, ball, rig, path, dt, t, Kp, Kd, kRoll, maxTilt));
    ctrl.UserData = [0 0 0 0];

    % Simulation; contact tolerances of the order of the water spacing and
    % steps short enough for the water
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.Settings("LengthScale", d, "Threads", 8));

    % Record the centre of mass of the water from the centre of the ball
    dt = 0.01;
    n = 2500;
    offset = zeros(n, 2);
    for k = 1:n
        sim.step(dt, 10);
        offset(k, :) = mean(water.CurrentVertices(:, 1:2)) - ball.Position(1:2);

        if mod(k, 10) == 0
            err = ball.Position(1:2) - reference(path, sim.Time);
            viewer.displayText(sprintf("err = [% .2f % .2f] m   |   water off centre = [% .1f % .1f] cm", ...
                err, 100*offset(k, :)));
        end
    end
    delete(sim);

    % Plot the ball position against its path, the water in the ball and
    % the motor torques over time
    clf(figure(2));
    bp = logBall.getChannel(1);            % ball Position [x y z]
    ref = cell2mat(arrayfun(@(t) reference(path, t), logBall.Time(:), "UniformOutput", false));
    torque = zeros(numel(logDrive.Time), 2);
    for i = 1:2                            % torque on the crank about the motor axis
        torque(:, i) = -logDrive.getChannel(i)*rig.axis(i, :)';
    end
    tiles = tiledlayout(3, 1);
    nexttile;
    plot(logBall.Time, bp(:, 1:2), "LineWidth", 1.3);
    hold on; set(gca, "ColorOrderIndex", 1);
    plot(logBall.Time, ref, ":", "LineWidth", 1.3);
    grid on; ylabel("ball pos [m]"); legend("x", "y", "x path", "y path");
    title(sprintf("Water ball from stop to stop (Kp = %.1f, Kd = %.1f)", Kp, Kd));
    nexttile;
    plot((1:n)*dt, offset*100, "LineWidth", 1.3);
    grid on; ylabel("water off centre [cm]"); legend("x", "y");
    nexttile;
    plot(logDrive.Time, torque/1e3, "LineWidth", 1.3);
    grid on; xlabel("time [s]"); ylabel("motor torque [kN m]"); legend("X drive", "Y drive");
    linkaxes(findobj(tiles, "Type", "axes"), "x");
    xlim([0 n*dt]);

end

function balanceLaw(fcn, ball, rig, path, dt, t, Kp, Kd, kRoll, maxTilt)
% PD control law of phxex_balance, tracking the path: the acceleration of
% the path plus the PD terms of the ball's in-plane position and velocity
% error give the plate tilt that makes the ball follow it (the ball
% accelerates as kRoll*sin(theta) on an incline). The tilt is given as
% the horizontal part [nx ny] of the plate normal: the ball accelerates
% along it. The commanded tilt follows it as a critically damped second
% order system with the time constant tau, so that neither the tilt nor
% its rate jumps and the heavy plate is not jerked. Each motor then
% follows the crank angle of that tilt by velocity, with a position
% feedback.

    tau = 0.1;                                  % time constant of the tilt (s)
    Ks = 50;                                    % servo position feedback (1/s)
    pos = ball.Position;
    vel = ball.LinearVelocity;

    % desired in-plane acceleration -> plate tilt
    [pRef, vRef, aRef] = reference(path, t);
    aDes = aRef + Kp*(pRef - pos(1:2)) + Kd*(vRef - vel(1:2));
    lim = sin(maxTilt);
    want = max(min(aDes/kRoll, lim), -lim);

    % Move towards that tilt
    tilt = fcn.UserData(1:2);
    rate = fcn.UserData(3:4);
    rate = rate + ((want - tilt)/tau^2 - 2*rate/tau)*dt;
    next = tilt + rate*dt;
    fcn.UserData = [next rate];

    phi0 = crankAngles(rig, tilt);
    phi1 = crankAngles(rig, next);
    for i = 1:2
        rig.drive(i).TargetVelocity = (phi1(i) - phi0(i))/dt + Ks*(phi0(i) - rig.drive(i).Angle);
    end
end

function path = randomStops(start, n, aMax, rest)
% n stops at random on the plate, each at least 1 m from the one before,
% after a rest at the start; each move takes as long as the peak
% acceleration aMax allows and is followed by a rest (s). The stream of
% random numbers has a fixed seed, so every run takes the same path.

    rs = RandStream("twister", "Seed", 25);
    W = start;
    while size(W, 1) <= n
        q = 2.6*rand(rs, 1, 2) - 1.3;
        if norm(q - W(end, :)) > 1
            W(end+1, :) = q;    %#ok<AGROW>
        end
    end
    path.W = W;                                         % the stops
    path.T = sqrt(4*vecnorm(diff(W), 2, 2)/aMax);       % durations of the moves
    path.start = cumsum([rest; path.T(1:end-1) + rest]); % their start times
end

function [p, v, a] = reference(path, t)
% Position, velocity and acceleration of the ball on the path at time t,
% in the plane: full acceleration to the middle of each move, full
% braking after it

    v = [0 0];
    a = [0 0];
    k = find(t >= path.start, 1, "last");
    if isempty(k)                               % the rest at the start
        p = path.W(1, :);
        return
    end
    A = path.W(k, :);
    d = path.W(k + 1, :) - A;
    T = path.T(k);
    s = t - path.start(k);
    if s < T/2
        p = A + 2*(s/T)^2*d;
        v = 4*s/T^2*d;
        a = 4/T^2*d;
    elseif s < T
        p = A + (1 - 2*((T - s)/T)^2)*d;
        v = 4*(T - s)/T^2*d;
        a = -4/T^2*d;
    else
        p = A + d;
    end
end

% ---------------------------------------------------------------------------
function rig = buildPlatform(ax, plateSize, maxTilt)
% A 2-DOF tilting platform. The plate sits on a column on a ball joint
% whose twist is locked, so it tilts every way but does not turn about the
% vertical. Two servo motors on the base, under the X and the Y edge, turn
% cranks; a push rod with a ball joint at each end lifts or lowers the
% plate edge above its crank.

    rig.pivot = [0 0 -plateSize(3)/2];          % tilting centre, under the plate
    rig.dir   = [1 0 0; 0 1 0];                 % crank and push point directions
    rig.a    = 1.4;                            % push points from the plate centre (m)
    rig.r     = 0.55;                           % crank (m)
    rig.L     = 0.9;                            % push rod (m)
    zBase     = rig.pivot(3) - rig.L - 0.5;     % top of the base
    shaft     = 0.27;                           % motor centre to crank (m)
    frameColor = [0.3 0.32 0.36];
    ez = [0 0 1];

    rig.plate = phx.Body(ax, "Position", [0 0 0], ...
        "Shape", {"Box", "Size", plateSize, "Color", 1, "Texture", "xor", "TextureBlend", 0.5}, "Density", 250);

    phx.Body(ax, "Type", "static", "Position", [0 0 zBase - 0.05], ...
        "Shape", {"Box", "Size", [3.2 3.2 0.1], "Color", [0.75 0.75 0.72], "Texture", "tiles", "TextureBlend", 0.2});
    hColumn = rig.pivot(3) - 0.05 - zBase;
    column = phx.Body(ax, "Type", "static", "Position", [0 0 zBase + hColumn/2], ...
        "Shape", {"Cylinder", "Diameter", 0.3, "Height", hColumn, "Color", frameColor});
    phx.SphericalJoint(column, rig.plate, "PointA", [0 0 hColumn/2 + 0.05], "PointB", rig.pivot, ...
        "TwistLimits", 0, "ConeAngle", 1.5*maxTilt, "Visible", false);

    for i = 1:2
        c = rig.dir(i, :);
        w =cross(c, ez);                       % motor axis: a positive turn lifts the crank tip
        R = [c' cross(ez, c)' ez'];             % crank frame: X along the crank, -Y along w
        M = rig.pivot + (rig.a - rig.r)*c - rig.L*ez;   % motor axis point
        T = M + rig.r*c;                        % crank tip, crank horizontal
        P = rig.pivot + rig.a*c;                % push point under the plate
        rig.M(i, :) = M;
        rig.axis(i, :) = w;

        % Motor beside the crank, its shaft on the axis through M
        housing = M - shaft*w;
        phx.Body(ax, "Type", "static", "Position", [housing(1:2) (zBase + M(3) - 0.15)/2], "Orientation", R, ...
            "Shape", {"Box", "Size", [0.25 0.25 M(3) - 0.15 - zBase], "Color", frameColor});
        motor = phx.Body(ax, "Type", "static", "Position", housing, "Orientation", [c' ez' w'], ...
            "Shape", {"Cylinder", "Diameter", 0.4, "Height", 0.4, "Color", [0.2 0.2 0.22]});
        crank = phx.Body(ax, "Position", (M + T)/2, "Orientation", R, "Collisions", false, ...
            "Shape", {"Box", "Size", [rig.r + 0.12, 0.1, 0.12], "Color", [0.95 0.55 0.1]}, "Density", 2000);
        rig.drive(i) = phx.RevoluteJoint(motor, crank, "PointA", [0 0 shaft], "PointB", [-rig.r/2 0 0], ...
            "AxisA", [0 0 1], "AxisB", [0 -1 0], "MaxTorque", 2e4, "Visible", false);

        rod = phx.Body(ax, "Position", (T + P)/2, "Collisions", false, ...
            "Shape", {"Cylinder", "Diameter", 0.06, "Height", rig.L, ...
            "Color", [0.85 0.86 0.88], "Material", "metal"}, "Density", 2000);
        phx.SphericalJoint(crank, rod, "PointA", [rig.r/2 0 0], "PointB", [0 0 -rig.L/2], "Visible", false);
        phx.SphericalJoint(rod, rig.plate, "PointA", [0 0 rig.L/2], "PointB", P, "Visible", false);
    end
end

function phi = crankAngles(rig, tilt)
% Crank angles (rad, from horizontal) that tilt the plate to the normal
% with the horizontal part tilt = [nx ny]. The plate turns without twist:
% about the axis ez x n, by the angle between ez and n.

    n = [tilt sqrt(1 - sum(tilt.^2))];
    v = [-n(2) n(1) 0];                         % ez x n
    V = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    R = eye(3) + V + V*V/(1 + n(3));            % rotation taking ez to n
    phi = zeros(1, 2);
    for i = 1:2
        c = rig.dir(i, :);
        d = rig.pivot + rig.a*c*R' - rig.M(i, :);   % push point from the motor axis
        E = dot(d, c);
        F = d(3);
        G = (dot(d, d) + rig.r^2 - rig.L^2)/(2*rig.r);
        phi(i) = atan2(F, E) - acos(G/hypot(E, F));
    end
end
