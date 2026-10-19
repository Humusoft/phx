function phxex_seaplate(duration, maxAmplitude, balance)
% PHXEX_SEAPLATE Kinova arm on a pontoon balancing a ball on a plate in a rising sea
%
% A Kinova Gen3 arm, imported from Robotics System Toolbox, stands on a
% wooden pontoon and holds a plate with a ball on it. The pontoon starts
% aground. The water rises, lifts it off the seabed and rocks it, and the
% waves keep growing for the whole run.
%
% Two control layers work in one phx.Function (see armControl below). The
% first one keeps the plate level and still in the world while the pontoon
% pitches and rolls under it. The second one tilts the plate to roll the
% ball back to the centre. The arm is a real one with real limits: its
% joints turn at most 1.4 rad/s, and it learns how the pontoon moves only
% through a lagging sensor. Sooner or later a wave group comes that it
% cannot follow, and the ball goes overboard.
%
% Input Arguments:
%     duration     - simulated time in seconds
%     maxAmplitude - wave amplitude reached at the end of the run in m
%     balance      - tilt the plate to keep the ball centered; without it
%                    the arm only holds the plate level
%
% Example:
%     phxex_seaplate                  % the ball holds until the sea gets rough
%     phxex_seaplate(80, 0.15, false) % plate only held level, the ball rolls
%                                     % off before the pontoon floats
%     phxex_seaplate(80, 0.06)        % a sea the arm can always handle
%
% Requires Robotics System Toolbox.
%
% See also phx.assembly.import, phx.Buoyancy, phx.Function

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        duration (1, 1) double {mustBePositive} = 40
        maxAmplitude (1, 1) double {mustBeNonnegative} = 0.16
        balance (1, 1) logical = true
    end

    % Pontoon, sea floor and the sea
    pontoon = [1.6 1.2 0.3];
    density = 600;                  % kg/m^3, floats at 60 % of its height
    keel = 0.3;                     % water under the keel once the sea is up
    riseTime = 15;                  % s for the water to come up
    startAmplitude = 0.03;
    rampTime = 70;                  % s for the waves to grow to maxAmplitude

    % Plate and ball
    plateR = 0.2;
    plateT = 0.012;
    ballR = 0.08;
    ballStart = [0.08 0.05];        % initial offset from the plate centre

    % Where the arm holds the plate, in the frame of its base
    holdAt = [0.4 0 0.7];

    % --- Pose of the arm --------------------------------------------------
    % Plate face up at holdAt, in the configuration farthest from the limits
    % of the three bounded joints, so that it has room to move every way
    robot = loadrobot("kinovaGen3", "DataFormat", "row");
    ik = inverseKinematics("RigidBodyTree", robot);
    limits = [pi 2.2 pi 2.5656 pi 2.05 pi];
    seeds = (rand(RandStream("twister", "Seed", 2), 40, 7) - 0.5).*limits*1.4;
    margin = -inf;
    for k = 1:size(seeds, 1)
        [q, info] = ik("EndEffector_Link", trvec2tform(holdAt), ones(1, 6), seeds(k, :));
        q = wrapAngle(q);
        m = min(limits([2 4 6]) - abs(q([2 4 6])));
        if info.PoseErrorNorm < 1e-3 && m > margin
            margin = m;
            q0 = q;
        end
    end
    Thold = getTransform(robot, q0, "EndEffector_Link");

    % --- Scene ------------------------------------------------------------
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0.1 0 0.4], ...
        "DefaultCameraPosition", [2.2 -3.2 1.6]*0.8);

    draft = density/1000*pontoon(3);
    seabed = -draft - keel;
    seabedBody = phx.Body(ax, "Type", "static", "Position", [0 0 seabed - 0.1], ...
        "Shape", {"Box", "Size", [8 8 0.2], "Texture", "tiles"});
    pont = phx.Body(ax, "Position", [0 0 seabed + pontoon(3)/2], ...
        "Shape", {"Box", "Size", pontoon, "Density", density, "Color", [0.75 0.55 0.35], ...
        "Texture", "wood", "TextureBlend", 0.5});

    % The arm is bolted to the deck by a joint; the tree gives its base no
    % mass of its own
    [bodies, joints] = phx.assembly.import(ax, robot, "Configuration", q0, ...
        "Position", pont.Position + [0 0 pontoon(3)/2]);
    base = bodies.base_link;
    base.Mass = 20;
    base.Inertia = [0.2 0.2 0.2];
    phx.FixedJoint(pont, base, "PointA", [0 0 pontoon(3)/2], "PointB", [0 0 0], ...
        "Visible", false);
    motor = [joints.Actuator1 joints.Actuator2 joints.Actuator3 joints.Actuator4 ...
             joints.Actuator5 joints.Actuator6 joints.Actuator7];
    [motor.MaxTorque] = deal(60);

    % The plate sits on the flange, which the tree calls EndEffector_Link;
    % the last body of the tree carries it as a fixed offset
    hand = bodies.Bracelet_Link;
    Ttool = hand.Transform\(base.Transform*Thold);
    Tplate = base.Transform*Thold*trvec2tform([0 0 plateT/2]);
    plate = phx.Body(ax, "Position", Tplate(1:3, 4)', "Orientation", Tplate(1:3, 1:3), ...
        "Shape", {"Cylinder", "Radius", plateR, "Height", plateT, "Density", 1800, ...
        "Color", [0.85 0.87 0.9], "Texture", "checker", "TextureBlend", 0.3}, "Friction", [0.6 0 0]);
    phx.FixedJoint(hand, plate, "TransformA", hand.Transform\Tplate, "TransformB", eye(4), ...
        "Visible", false);
    p = Tplate*[ballStart, plateT/2 + ballR, 1]';
    ball = phx.Body(ax, "Position", p(1:3)', ...
        "Shape", {"Sphere", "Radius", ballR, "Density", 1000, "Color", [0.9 0.3 0.25], ...
        "Texture", "checker", "TextureBlend", 0.5}, "Friction", [0.6 0.002 0]);

    % Short-crested sea: one dominant swell and two smaller ones from other
    % headings, over a mean level that rises from below the hull to 0
    swell = [1.00 6.0   0   0       % relative amplitude, wavelength, heading deg, phase
             0.35 4.0  50   1.3
             0.25 3.0 -35   2.1];
    level0 = seabed - 0.05;
    meanLevel = @(t) level0*(1 - min(t/riseTime, 1));
    amp = @(t) startAmplitude + (maxAmplitude - startAmplitude)*min(t/rampTime, 1);
    wave = @(x, y, t) meanLevel(t) + amp(t).*swellHeight(x, y, t, swell);
    phx.Buoyancy(pont, "LevelFunction", wave, "LinearDamping", 800, "AngularDamping", 300, ...
        "SurfaceSize", [8 8], "SurfaceStep", 0.25);

    % --- Controller -------------------------------------------------------
    % A rolling solid ball accelerates at (5/7)*g per radian of tilt; the
    % ball loop is placed at 0.8 rad/s with damping 0.7
    gBall = 5/7*9.81;
    wn = 0.8;
    zeta = 0.7;
    ctrl = struct("base", base, "hand", hand, "Ttool", Ttool, ...
        "Thold", base.Transform*Thold, "zBase0", base.Position(3), "q0", q0, ...
        "plate", plate, "ball", ball, "plateR", plateR, "balance", balance, ...
        "Kp", wn^2/gBall, "Kd", 2*zeta*wn/gBall, "tiltMax", deg2rad(8), ...
        "gain", 4.5, "lag", 0.4, "heave", 0.5, "heaveTau", 3, "Vmax", 1.4);
    phx.Function(motor, @(fcn, ~, dt, ~) armControl(fcn, motor, dt, ctrl));

    % Plate pose and ball position, recorded in the background for the plot
    logPlate = phx.Logger(plate, "Parameters", "Transform", "Frequency", 50);
    logBall = phx.Logger(ball, "Parameters", "Position", "Frequency", 50);

    % Bodies in creation order, base first, which suits the joint chain; the
    % joints need the exact (dantzig) solver to run their motors at the
    % commanded rates
    arm = struct2cell(bodies);
    arm = [arm{:}];
    sim = phx.Simulation([seabedBody pont arm plate ball], "EngineSettings", ...
        phx.engine.BulletSettings("Margin", 0.002, "Solver", "dantzig"));

    % --- Run --------------------------------------------------------------
    dt = 0.0025;
    subSteps = 5;
    zStart = pont.Position(3);
    liftOff = NaN;
    dropped = NaN;
    while sim.Time < duration
        sim.step(dt*subSteps, subSteps, subSteps);
        t = sim.Time;

        e = plate.Transform\[ball.Position 1]';
        off = norm(e(1:2));
        if isnan(liftOff) && pont.Position(3) > zStart + 0.01
            liftOff = t;
        end
        if isnan(dropped) && (off > plateR || e(3) < 0)
            dropped = t;
        end

        if isnan(liftOff)
            stage = "aground, the water is rising";
        elseif isnan(dropped)
            stage = sprintf("afloat, ball %3.0f mm off centre", 1000*off);
        else
            stage = sprintf("the ball went overboard at %.1f s", dropped);
        end
        viewer.displayText(sprintf("t = %4.1f s   waves %.3f m   %s", t, amp(t), stage));

        if t > dropped + 3
            break
        end
    end

    % Ball offset from the plate centre, in the plane of the plate: each row
    % of the plate log is its 4x4 transform by columns, so 1:3 and 5:7 are
    % the in-plane axes and 13:15 the centre
    tLog = logPlate.Time;
    Tp = logPlate.Data;
    d = logBall.Data - Tp(:, 13:15);
    offLog = hypot(sum(d.*Tp(:, 1:3), 2), sum(d.*Tp(:, 5:7), 2));
    delete(sim);

    % --- Result -----------------------------------------------------------
    if isnan(dropped)
        fprintf("The arm kept the ball on the plate for the whole run, up to %.3f m waves.\n", amp(t));
    elseif isnan(liftOff) || dropped < liftOff
        fprintf("The ball rolled off at %.1f s, before the pontoon was even afloat.\n", dropped);
    else
        fprintf("Afloat at %.1f s, the ball went overboard at %.1f s in %.3f m waves.\n", ...
            liftOff, dropped, amp(dropped));
    end

    clf(figure(2));
    yyaxis left
    plot(tLog, 1000*offLog, "LineWidth", 1.2);
    ylim([0 1100*plateR]);
    yline(1000*plateR, "--", "plate edge");
    ylabel("ball off centre [mm]");
    yyaxis right
    plot(tLog, amp(tLog), "LineWidth", 1.2);
    ylabel("wave amplitude [m]");
    if ~isnan(liftOff)
        xline(liftOff, ":", "afloat");
    end
    grid on
    xlabel("time [s]");
    title("Ball on the plate in a growing sea");

end

function armControl(fcn, motor, dt, c)
% Both control layers. The ball layer turns the ball offset into a plate
% tilt; the arm layer turns the plate pose error into joint rates through the
% geometric Jacobian, after taking away the motion the pontoon gives the
% plate by itself. That motion reaches the controller through a first-order
% lag, as from a filtered motion sensor. The plate height follows the slow
% rise of the pontoon fully and its wave heave only half, so the plate rides
% the swell a little instead of hanging nailed in space.
    n = numel(motor);
    q = [motor.Angle];
    s = fcn.UserData;
    Tb = c.base.Transform;
    Tt = c.hand.Transform*c.Ttool;
    pt = Tt(1:3, 4);

    % Ball layer: PD on the ball position and velocity in the plate frame
    Rwant = c.Thold(1:3, 1:3);
    if c.balance
        Tp = c.plate.Transform;
        Rp = Tp(1:3, 1:3);
        r = c.ball.Position(:) - Tp(1:3, 4);
        v = c.ball.LinearVelocity(:) - c.plate.LinearVelocity(:) ...
            - cross(c.plate.AngularVelocity(:), r);
        e = Rp'*r;
        ev = Rp'*v;
        if norm(e(1:2)) < c.plateR
            % Raise the side the ball is on
            tilt = [c.Kp*e(2) + c.Kd*ev(2), -(c.Kp*e(1) + c.Kd*ev(1))];
            tilt = tilt*min(1, c.tiltMax/norm(tilt));
            Rwant = Rwant*rotx(tilt(1))*roty(tilt(2));
        end
    end

    % Pontoon motion as the lagging sensor reports it
    wb = c.base.AngularVelocity(:);
    vb = c.base.LinearVelocity(:);
    zb = Tb(3, 4);
    if isempty(s)
        s = struct("wb", wb, "vb", vb, "zSlow", zb);
    end
    a = dt/(c.lag + dt);
    s.wb = s.wb + a*(wb - s.wb);
    s.vb = s.vb + a*(vb - s.vb);
    s.zSlow = s.zSlow + dt/(c.heaveTau + dt)*(zb - s.zSlow);
    fcn.UserData = s;

    % Arm layer: wanted plate twist in the world, less what the pontoon does
    zWant = c.Thold(3, 4) + (s.zSlow - c.zBase0) + c.heave*(zb - s.zSlow);
    v = c.gain*([c.Thold(1:2, 4); zWant] - pt) - (s.vb + cross(s.wb, pt - Tb(1:3, 4)));
    v(3) = v(3) + c.heave*s.vb(3) + (1 - c.heave)*(zb - s.zSlow)/c.heaveTau;
    w = c.gain*rotationVector(Rwant*Tt(1:3, 1:3)') - s.wb;

    % Geometric Jacobian in the world, read off the simulated links: each
    % joint turns the plate about the Z axis of its frame on the inner link
    J = zeros(6, n);
    for k = 1:n
        Tj = motor(k).Parents{1}.Transform*motor(k).TransformA;
        J(:, k) = [Tj(1:3, 3); cross(Tj(1:3, 3), pt - Tj(1:3, 4))];
    end

    % Joint rates by damped least squares; the redundant seventh joint is
    % drawn back toward the starting configuration
    Jinv = J'/(J*J' + 0.02^2*eye(6));
    qd = Jinv*[w; v] + (eye(n) - Jinv*J)*2*wrapAngle(c.q0 - q)';
    qd = max(-c.Vmax, min(c.Vmax, qd));
    for k = 1:n
        motor(k).TargetVelocity = qd(k);
    end
end

function z = swellHeight(x, y, t, swell)
% Sum of the swell components at points x, y (any size) and time t.
    z = 0;
    for k = 1:size(swell, 1)
        kw = 2*pi/swell(k, 2);
        heading = deg2rad(swell(k, 3));
        z = z + swell(k, 1)*sin(kw*(x*cos(heading) + y*sin(heading)) - sqrt(9.81*kw)*t + swell(k, 4));
    end
end

function r = rotationVector(R)
% Axis times angle of a rotation matrix.
    th = acos(max(-1, min(1, (trace(R) - 1)/2)));
    if th < 1e-9
        r = [0; 0; 0];
    else
        r = th/(2*sin(th))*[R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    end
end

function R = rotx(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = roty(a)
    R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function a = wrapAngle(a)
% Joint.Angle is wrapped to [-pi, pi], so the errors are wrapped too.
    a = mod(a + pi, 2*pi) - pi;
end