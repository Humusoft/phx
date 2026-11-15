function phxex_delta(opts)
% PHXEX_DELTA Delta robot stacking blocks out over the edge of a pedestal
%
% A three-arm delta robot. Each motor (phx.RevoluteJoint) turns an upper
% arm, and a parallelogram of two rods on phx.SphericalJoint pins carries
% the travelling plate, which keeps its orientation wherever it goes.
%
% The robot stacks blocks as in the book-stacking problem: each block is
% shifted so that the blocks above it keep their centre of mass just
% inside the edge below. Four blocks go up at a given fraction of that
% limit, the top one reaching entirely beyond the pedestal, and a fifth is
% set squarely on top of them.
%
% A phx.Function turns the path into arm angles by inverse kinematics,
% drives the motors by velocity with a position feedback, and works the
% suction cup under the plate. The cup holds a block by a force, not by a
% joint: the vacuum pulls the block against the cup, and only friction
% keeps it from sliding sideways.
%
% Figure 2 shows how far the centres of mass of the measured stack lie
% beyond the edges, the friction the cup needs against what it has, and
% the motor torques.
%
% phxex_delta("Overhang", 1.02) builds past the limit.
% phxex_delta("Acceleration", 1.5) moves harder than the cup's friction
% can carry a block.
%
% See also phx.RevoluteJoint, phx.SphericalJoint, phx.Function

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        opts.Overhang     (1, 1) double {mustBePositive} = 0.97   % shift of the four blocks, fraction of the limit
        opts.Acceleration (1, 1) double {mustBePositive} = 1      % peak plate acceleration of a move (g)
    end

    geo.z0   = 1.05;                 % height of the motor axes (m)
    geo.Rb   = 0.2;                  % radius of the motor axes (m)
    geo.Re   = 0.06;                 % radius of the plate pins (m)
    geo.L1   = 0.35;                 % upper arm (m)
    geo.L2   = 0.8;                  % rod (m)
    geo.s    = 0.1;                  % rod spacing of a parallelogram (m)
    geo.phi  = [0 2/3 4/3]*pi;       % arm azimuths
    geo.cupH = 0.04;                 % suction cup below the plate (m)
    maxTorque = 40;                  % motor torque limit (N*m)
    vac.force = 30e3*pi*0.01^2;      % 30 kPa on a 20 mm cup (N)
    vac.reach = [0.025 0.003];       % seal holds within this offset and gap (m)
    vac.half  = 0.025;               % half the block height (m)

    % Blocks: a row on the table, stacked out over the -Y edge of a pedestal.
    % Block k of four sits L/(2*(5-k)) beyond the one below, times Overhang;
    % the fifth sits squarely on the fourth.
    L  = 0.08;  hB = 2*vac.half;  n = 5;
    yP = 0.05;  hP = 0.08;           % pedestal centre and height (m)
    face = 0.015 + geo.cupH;         % plate centre to the cup face
    y = yP - cumsum(opts.Overhang*L./(2*(4:-1:1)));
    picks  = [((1:n)' - 3)*0.1, 0.25*ones(n, 1), (hB + 0.001 + face)*ones(n, 1)];
    places = [zeros(n, 1), [y y(end)]', hP + (1:n)'*hB + face];   % set down, not dropped
    zSafe  = hP + n*hB + 0.03 + face;
    path = stackingPath([0 0 zSafe], picks, places, zSafe, opts.Acceleration*9.81);
    for k = 1:size(path.W, 1)        % every waypoint must be within reach
        deltaIK(geo, path.W(k, :));
    end

    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [2.0 -0.45 0.6], ...
        "DefaultCameraTarget", [0 0 0.6], "Texture", "checker");

    robot = buildRobot(ax, geo, [0 0 zSafe], maxTorque);
    phx.Body(ax, "Type", "static", "Position", [0 yP hP/2], ...
        "Shape", {"Box", "Size", [L L hP], "Color", [0.3 0.32 0.36]});
    colors = [0.85 0.3 0.25; 0.95 0.75 0.2; 0.3 0.6 0.85; 0.4 0.7 0.35; 0.6 0.4 0.75];
    for k = 1:n
        blocks(k) = phx.Body(ax, "Position", [picks(k, 1:2) hB/2], "Friction", 0.5, ...
            "Shape", {"Box", "Size", [L L hB], "Density", 800, "Color", colors(k, :), ...
            "Texture", "wood", "TextureBlend", 0.3}); %#ok<AGROW>
    end

    ctrl = phx.Function([robot.arm robot.cup], ...
        @(fcn, ~, dt, t) controller(fcn, robot, blocks, geo, vac, path, dt, t));
    ctrl.UserData = 0;

    logPlate = phx.Logger(robot.plate, "Parameters", "LinearAcceleration", "Frequency", 500);
    logDrive = phx.Logger(robot.drive, "Parameters", "TorqueA", "Frequency", 500);

    dt = 1e-3;
    sim = phx.Simulation(ax, "PacingRate", 1);
    P = [];                          % block positions just before the fifth is let go
    for k = 1:round((path.t(end) + 2)/(10*dt))
        sim.step(10*dt, 10);
        if isempty(P) && sim.Time >= path.release(end) - 10*dt
            P = cell2mat(arrayfun(@(b) b.Position, blocks(:), "UniformOutput", false));
        end
    end
    fell = cell2mat(arrayfun(@(b) b.Position(3), blocks(:), "UniformOutput", false)) ...
        < places(:, 3) - face - hB;

    t = logPlate.Time;               % start of the step whose end state is logged
    a = logPlate.getChannel(1);      % = acceleration of the held block
    torque = zeros(numel(t), 3);
    for i = 1:3                      % torque on the arm about its motor axis
        torque(:, i) = -logDrive.getChannel(i)*[-sin(geo.phi(i)); cos(geo.phi(i)); 0];
    end
    delete(sim);

    % How far the centre of mass of each block and all above it lies beyond
    % the edge under it, from the measured stack: positive = it tips
    s = yP - P(:, 2);                % position along the overhang
    support = [0; s(1:4)];
    margin4 = arrayfun(@(k) mean(s(k:4)) - support(k) - L/2, 1:4);
    margin5 = arrayfun(@(k) mean(s(k:5)) - support(k) - L/2, 1:4);

    % Friction the cup needs to carry the block sideways, against what it has
    m  = blocks(1).Mass;
    mu = blocks(1).Friction*robot.cup.Friction;
    pressing = vac.force - m*(9.81 + a(:, 3));                 % block against the cup
    need = m*vecnorm(a(:, 1:2), 2, 2)./pressing;
    need(~arrayfun(@(tk) vacuumAt(path, tk), t)) = NaN;

    clf(figure(2));
    tiledlayout(3, 1);
    nexttile;
    bar(1:4, [margin4' margin5']*1e3);
    yline(0, "-", "edge", "LabelHorizontalAlignment", "left");
    grid on; ylim([-10 10]);
    legend("four blocks", "with the fifth on top");
    xlabel("block, with all above it"); ylabel("beyond the edge [mm]");
    if any(fell)
        outcome = "blocks " + strjoin(string(find(fell)'), ", ") + " fell";
    else
        outcome = "all standing";
    end
    title(sprintf("Overhang %.0f %% of the limit: %s", 100*opts.Overhang, outcome));
    tCup = nexttile;
    plot(t, need, "LineWidth", 1); grid on;
    yline(mu, ":", "available", "LabelHorizontalAlignment", "left");
    ylim([0 1]);
    ylabel("friction needed"); title(sprintf("Suction cup, moves at %.1f g", opts.Acceleration));
    tMotor = nexttile;
    plot(t, torque, "LineWidth", 1); grid on;
    yline([-1 1]*maxTorque, ":", "limit", "LabelHorizontalAlignment", "left");
    ylabel("motor torque [N m]"); xlabel("time [s]");
    legend("arm 1", "arm 2", "arm 3");
    linkaxes([tCup tMotor], "x");
end

% ---------------------------------------------------------------------------
function controller(fcn, robot, blocks, geo, vac, path, dt, t)
% One substep of the robot controller: the motors, then the suction cup

    Kp = 100;                                    % position feedback (1/s)
    th0 = deltaIK(geo, pathAt(path, t));
    th1 = deltaIK(geo, pathAt(path, t + dt));
    for i = 1:3
        xa = robot.arm(i).XAxis;
        th = atan2(-xa(3), xa(1)*cos(geo.phi(i)) + xa(2)*sin(geo.phi(i)));
        robot.drive(i).TargetVelocity = (th1(i) - th0(i))/dt + Kp*(th0(i) - th);
    end

    % The cup pulls the block under it while the vacuum is on and the seal
    % holds. UserData: index of the held block, 0 = none, -1 = seal lost.
    held = fcn.UserData;
    cup = robot.cup;
    if ~vacuumAt(path, t)
        if held > 0
            blocks(held).AppliedForce = [0 0 0];
            cup.AppliedForce = [0 0 0];
        end
        fcn.UserData = 0;
        return
    elseif held < 0
        return
    end
    face = cup.Position - [0 0 geo.cupH/2];
    if held == 0
        for k = 1:numel(blocks)
            if sealed(blocks(k), face, vac)
                held = k;
                break
            end
        end
        if held == 0
            return
        end
    elseif ~sealed(blocks(held), face, vac)
        blocks(held).AppliedForce = [0 0 0];
        cup.AppliedForce = [0 0 0];
        fcn.UserData = -1;
        return
    end
    blocks(held).AppliedForce = [0 0 vac.force]*blocks(held).Orientation;   % body frame
    cup.AppliedForce = [0 0 -vac.force]*cup.Orientation;
    fcn.UserData = held;
end

function ok = sealed(block, face, vac)
% True when the top face of the block lies under the cup, close enough to seal

    d = block.Position + block.ZAxis*vac.half - face;
    ok = norm(d(1:2)) < vac.reach(1) && abs(d(3)) < vac.reach(2);
end

% ---------------------------------------------------------------------------
function path = stackingPath(start, picks, places, zSafe, aMax)
% Moves of the plate centre from block to block, each with peak acceleration aMax

    dwell = 0.15;                    % time to grip or release (s)
    W = start;
    durations = zeros(1, 0);
    vacuum = false(1, 0);
    for k = 1:size(picks, 1)
        % target, vacuum on during the move
        moves = {[picks(k, 1:2) zSafe],  false
                 picks(k, :),            false
                 picks(k, :),            true
                 [picks(k, 1:2) zSafe],  true
                 [places(k, 1:2) zSafe], true
                 places(k, :),           true
                 places(k, :),           false
                 [places(k, 1:2) zSafe], false};
        for m = 1:size(moves, 1)
            d = norm(moves{m, 1} - W(end, :));
            if d == 0
                T = dwell;
            else
                T = sqrt(2*pi*d/aMax);   % cycloidal profile: peak acceleration 2*pi*d/T^2
            end
            W(end+1, :) = moves{m, 1};   %#ok<AGROW>
            durations(end+1) = T;        %#ok<AGROW>
            vacuum(end+1) = moves{m, 2}; %#ok<AGROW>
        end
    end
    path.W = W;
    path.t = [0 cumsum(durations)];
    path.vacuum = vacuum;
    path.release = path.t(7:8:end);  % start of each release
end

function p = pathAt(path, t)
% Point of the path at time t: a cycloidal profile on each move, rest at the ends

    k = find(t >= path.t(1:end-1), 1, "last");
    if isempty(k)
        p = path.W(1, :);
    elseif t >= path.t(end)
        p = path.W(end, :);
    else
        tau = (t - path.t(k))/(path.t(k+1) - path.t(k));
        s = tau - sin(2*pi*tau)/(2*pi);
        p = path.W(k, :) + s*(path.W(k+1, :) - path.W(k, :));
    end
end

function on = vacuumAt(path, t)
% Vacuum state of the move running at time t

    k = find(t >= path.t(1:end-1), 1, "last");
    on = ~isempty(k) && t < path.t(end) && path.vacuum(k);
end

% ---------------------------------------------------------------------------
function robot = buildRobot(ax, geo, p0, maxTorque)
% Build the frame, the three arms with their parallelograms and the plate at p0

    ez = [0 0 1];
    theta = deltaIK(geo, p0);
    frameColor = [0.3 0.32 0.36];

    phx.Body(ax, "Type", "static", "Position", [0 0 -0.02], ...
        "Shape", {"Box", "Size", [1.6 1.6 0.04], "Color", [0.75 0.75 0.72], "Texture", "tiles", "TextureBlend", 0.2});
    phx.Body(ax, "Type", "static", "Position", [0 0 geo.z0 + 0.08], ...
        "Shape", {"Cylinder", "Diameter", 2*geo.Rb + 0.16, "Height", 0.03, "Color", frameColor});
    for a = geo.phi + pi/3           % posts between the arms, beams to the base
        u = [cos(a) sin(a) 0];
        phx.Body(ax, "Type", "static", "Position", 0.62*u + [0 0 (geo.z0 + 0.095)/2], ...
            "Shape", {"Cylinder", "Diameter", 0.05, "Height", geo.z0 + 0.095, "Color", frameColor});
        phx.Body(ax, "Type", "static", "Position", 0.45*u + [0 0 geo.z0 + 0.08], "Orientation", [u' [-u(2) u(1) 0]' ez'], ...
            "Shape", {"Box", "Size", [0.36 0.05 0.03], "Color", frameColor});
    end

    robot.plate = phx.Body(ax, "Position", p0, "Collisions", false, ...
        "Shape", {"Cylinder", "Diameter", 2*geo.Re + 0.06, "Height", 0.03, "Density", 1000, ...
        "Color", [0.85 0.86 0.88], "Material", "metal"});
    robot.cup = phx.Body(ax, "Position", p0 - [0 0 0.015 + geo.cupH/2], "Friction", 0.8, ...
        "Shape", {"Cylinder", "Diameter", 0.025, "Height", geo.cupH, "Density", 1000, "Color", [0.2 0.2 0.2]});
    phx.FixedJoint(robot.plate, robot.cup, "PointA", [0 0 -0.015 - geo.cupH/2], "Visible", false);

    for i = 1:3
        u = [cos(geo.phi(i)) sin(geo.phi(i)) 0];     % radial
        t = [-sin(geo.phi(i)) cos(geo.phi(i)) 0];    % along the motor axis
        B = geo.Rb*u + geo.z0*ez;                    % motor axis point

        motor = phx.Body(ax, "Type", "static", "Position", B, "Orientation", [u' t' ez'], ...
            "Shape", {"Box", "Size", [0.1 0.12 0.1], "Color", [0.2 0.2 0.22]});

        % Upper arm: body X from the motor axis to the elbow, Y along the motor axis
        xa  = cos(theta(i))*u - sin(theta(i))*ez;
        E   = B + geo.L1*xa;
        R   = [xa' t' cross(xa, t)'];
        arm = phx.Body(ax, "Position", (B + E)/2, "Orientation", R, "Collisions", false, ...
            "Shape", {"Box", "Size", [geo.L1 + 0.04, geo.s + 0.03, 0.025], "Density", 700, ...
            "Color", [0.95 0.55 0.1]});
        robot.arm(i) = arm;
        robot.drive(i) = phx.RevoluteJoint(motor, arm, "PointA", [0 0 0], "PointB", [-geo.L1/2 0 0], ...
            "AxisA", [0 1 0], "AxisB", [0 1 0], "MaxTorque", maxTorque, "Visible", false);

        % Parallelogram: two equal rods, elbow pins E +- s/2 t, plate pins p0 + Re u +- s/2 t
        for k = [-1 1]
            top = E + k*geo.s/2*t;
            bot = p0 + geo.Re*u + k*geo.s/2*t;
            rod = phx.Body(ax, "Position", (top + bot)/2, "ZAxis", top - bot, "Collisions", false, ...
                "Shape", {"Cylinder", "Diameter", 0.016, "Height", geo.L2, "Density", 1000, ...
                "Color", [0.15 0.15 0.15]});
            phx.SphericalJoint(arm, rod, "PointA", (top - (B + E)/2)*R, "PointB", [0 0 geo.L2/2], "Visible", false);
            phx.SphericalJoint(rod, robot.plate, "PointA", [0 0 -geo.L2/2], "PointB", bot - p0, "Visible", false);
        end
    end
end

% ---------------------------------------------------------------------------
function theta = deltaIK(geo, p)
% Arm angles below the horizontal (rad) that put the plate centre at p

    theta = zeros(1, 3);
    for i = 1:3
        u = [cos(geo.phi(i)) sin(geo.phi(i)) 0];
        d = (geo.Rb - geo.Re)*u + [0 0 geo.z0] - p;  % plate pin to motor axis
        E = 2*geo.L1*dot(d, u);
        F = -2*geo.L1*d(3);
        G = geo.L2^2 - dot(d, d) - geo.L1^2;
        c = G/hypot(E, F);
        if abs(c) > 1
            error("phx:delta:unreachable", "The point [%g %g %g] is out of reach.", p);
        end
        theta(i) = atan2(F, E) + acos(c);           % elbow out
    end
end
