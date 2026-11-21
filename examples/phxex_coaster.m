function phxex_coaster(Options)
% PHXEX_COASTER Roller-coaster train on a drop, a loop and a banked turn
%
% A train of three cars pushed over the crest rolls down a 14 m drop,
% through a loop and a 180 deg banked turn, and leaves the end of the track
% over a small kicker into a brick wall. Nothing guides the cars but their
% wheels on the rails: flanged wheels with a coned, compliant tread ride
% two tube rails (static triangle meshes), and the front axle of each car
% sits in a springy bushing that lets it roll and yaw, so that all four
% wheels follow the twist of the banked turn. The cars are coupled by
% stiff damped springs, colored by their force. The bank of the turn is
% set for the speed the train is expected to have there (see coasterTrack
% below). The camera stands in the middle of the ride and follows the
% middle car; a gauge shows its speed and a plot the load felt by a rider
% over the last five seconds.
%
% Options:
%     RecordFile - MPEG-4 file to record the camera view to (empty, the
%                  default, records nothing)
%
% Example:
%     phxex_coaster(RecordFile = "coaster.mp4")
%
% See also phx.Body, phx.shape.Mesh, phx.shape.Revolution, phx.Camera

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Options.RecordFile (1, 1) string = ""
    end

    gauge = 1.0;                 % distance of the rail axes (m)
    railRadius = 0.05;           % rail tube radius (m)
    wheelRadius = 0.15;          % wheel tread radius (m)
    speed0 = 3;                  % push over the crest (m/s)
    trk = coasterTrack("Height", 14, "LoopLength", 44, "TurnLength", 56, "Speed", speed0);

    [viewer, ax] = phx.extra.Viewer("clear", ...
        "DefaultCameraPosition", [10 -45 30], "DefaultCameraTarget", [40 10 4], Lighting="studio");

    phx.Body(ax, "Type", "static", "Position", [40 10 -0.1], ...
        "Shape", {"Box", "Size", [130 70 0.2], "Color", [0.55 0.7 0.45], ...
        "Texture", phx.base.Shape.tiles([2048 1024], [64 32]), "TextureBlend", 0.3});

    % The rails, and the spine, ties and columns that carry them (drawn only)
    railShape = {"Envelope", "concave", "Centered", false, ...
        "Color", [0.8 0.15 0.1], "Material", "metal"};
    for side = [1 -1]
        [V, F] = railMesh(trk, side*gauge/2, railRadius);
        % Low friction: the wheels only roll, and a flange pressed against
        % the round rail climbs it less easily
        phx.Body(ax, "Type", "static", "Friction", 0.05, ...
            "Shape", [{"Mesh", "Vertices", V, "Faces", F}, railShape]);
    end
    [V, F] = trackStructure(trk, gauge);
    phx.Body(ax, "Type", "static", "Collisions", false, "Shape", {"Mesh", "Vertices", V, ...
        "Faces", F, "Envelope", "box", "Centered", false, "Color", [0.85 0.85 0.8]});

    addSurroundings(ax, trk);

    % A wall of loose light bricks across the landing lane, 13 m past the end
    % of the track, where the flying train comes down
    phx.assembly.wall(ax, "Size", [6 0.3 3], "Rows", 6, "Columns", 6, ...
        "Position", [23.5 24.8 0], "EulerAngles", [0 0 pi/2], "Density", 600, ...
        "Color", [0.8 0.4 0.3], "RandomTint", 0.3);

    % The train: cars 2 m apart on the crest, the first one in front
    nCars = 3;
    colors = [0.15 0.35 0.85; 0.9 0.6 0.1; 0.2 0.65 0.3];
    for k = 1:nCars
        cars(k) = buildCart(ax, trk, 1.5 + 2*(nCars - k), colors(k, :), ...
            gauge, railRadius, wheelRadius, speed0); %#ok<AGROW>
    end
    % Couplings: stiff damped springs from the rear of a car to the front of the next
    for k = 1:nCars - 1
        phx.Spring(cars(k), cars(k + 1), "PointA", [-0.7 0 0], "PointB", [0.7 0 0], ...
            "FreeLength", 0.6, "Stiffness", 2e5, "Damping", 5e3, ...
            "Colormap", hot(64), "ColorRange", [0 2e4]);
    end
    cart = cars(ceil(nCars/2));

    % The camera stands on the ground inside the ride and follows the middle car
    camera = phx.Camera(cart, cart, "PointA", [-8 -5 3], ...
        "RecordFile", Options.RecordFile);

    % Speedometer and the load of the last few seconds, in the viewer's corner
    speedometer = uigauge(viewer.Figure, "Limits", [0 70], "Position", [10 10 140 140]);
    uilabel(viewer.Figure, "Text", "speed (km/h)", "Position", [10 150 140 20], ...
        "HorizontalAlignment", "center");
    loadAxes = uiaxes(viewer.Figure, "Position", [160 10 300 150], ...
        "XLim", [-5 0], "YLim", [-1 8], "XGrid", "on", "YGrid", "on");
    xlabel(loadAxes, "time (s)");
    ylabel(loadAxes, "load (g)");
    yline(loadAxes, 1, ":");
    loadAxes.Toolbar.Visible = "off";
    disableDefaultInteractivity(loadAxes);
    loadLine = line(loadAxes, NaN, NaN, "LineWidth", 1.5);
    history = NaN(2, 250);                       % [time; load], 5 s at 50 Hz

    % Contact tolerances for the decimetre-sized wheels and rails
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.Settings("LengthScale", 0.1, "Threads", 2));
    dt = 0.02;
    velocity = cart.LinearVelocity;
    while sim.Time < 17.5 && isvalid(viewer)
        sim.step(dt, 20);

        % Load felt by a rider: the mean acceleration over the step without
        % gravity, along the car's up axis (the wheels' impacts on the rails
        % average out)
        accel = (cart.LinearVelocity - velocity)/dt;
        velocity = cart.LinearVelocity;
        R = cart.Orientation;
        load = dot(accel + [0 0 9.81], R(:, 3))/9.81;
        history = [history(:, 2:end), [sim.Time; load]];
        set(loadLine, "XData", history(1, :) - sim.Time, "YData", history(2, :));
        speedometer.Value = min(3.6*norm(cart.LinearVelocity), 70);
    end
    delete(sim);
    delete(camera);                              % finalizes the video file
end

function addSurroundings(ax, trk)
% Palm trees outside the ride, clear of the camera's view of the track and
% of the lane where the car lands
    rng(0);                                      % reproducible palm placement
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    palm = phx.shape.Mesh("Source", resdir + "palm_tree.obj", "Material", "matte", ...
        "Centered", false, "Scale", 1.1);
    sites = [(-8:9:88)', -8 + 0*(-8:9:88)'           % behind the drop and the loop
             (36:9:90)', 33 + 0*(36:9:90)'           % behind the run-out
             92 + 0*(4:9:22)', (4:9:22)'];           % behind the turn
    for k = 1:size(sites, 1)
        position = sites(k, :) + 2*randn(1, 2);
        yaw = 2*pi*rand;
        % The collision hull of a palm reaches 4.9 m from its trunk
        if min(vecnorm(trk.P(:, 1:2) - position, 2, 2)) > 6.5
            phx.Body(ax, "Type", "static", "Position", [position 0], ...
                "EulerAngles", [0 0 yaw], "Shape", palm);
        end
    end
end

function cart = buildCart(ax, trk, s0, color, gauge, railRadius, wheelRadius, speed0)
% A platform on four flanged wheels; the front pair sits on a pivoting axle

    % Car pose at the arc length s0, its axes along the track frame
    i0 = find(trk.s >= s0, 1);
    R0 = [trk.T(i0, :)' trk.L(i0, :)' trk.U(i0, :)'];
    hAxle = railRadius + wheelRadius;            % axles above the rail axes
    pCart = trk.P(i0, :) + (hAxle + 0.05)*trk.U(i0, :);
    v0 = speed0*trk.T(i0, :);

    cart = phx.Body(ax, "Position", pCart, "Orientation", R0, ...
        "Shape", {"Box", "Size", [1.4 0.75 0.1], "Color", color}, ...
        "Mass", 400, "Inertia", 400/12*[0.75^2 + 0.1^2, 1.4^2 + 0.1^2, 1.4^2 + 0.75^2], ...
        "LinearVelocity", v0);

    % Wheel profile [axial radius]: tread over the rail, flange inside the gauge.
    % The tread grows 5 mm towards the flange: an axle that drifts sideways
    % rolls on a larger radius on one side and steers back, and the flange
    % stays off the rail.
    r = wheelRadius;
    profile = [-0.05 0; -0.05 r; 0.055 r + 0.005; 0.055 r + 0.06; 0.075 r + 0.06; 0.075 0];
    wheelShape = cell(1, 2);                     % left and right wheel
    for k = 1:2
        wheelShape{k} = phx.shape.Revolution("Axis", "y", "Envelope", "cones", ...
            "Profile", profile.*[2*k - 3, 1], "Density", 500, "Color", [0.3 0.3 0.3], ...
            "Texture", "stripes", "TextureBlend", 0.4);
    end

    for xw = [0.55 -0.55]
        host = cart;
        offset = [0 0 0];
        if xw > 0
            % Front axle: sprung and damped in roll and yaw (stops at 0.6 rad),
            % so the wheels follow the twist; fixed in translation and pitch
            offset = [xw 0 -0.05];
            host = phx.Body(ax, "Position", pCart + offset*R0', "Orientation", R0, ...
                "Shape", {"Box", "Size", [0.08 gauge - 0.2 0.06], "Color", [0.4 0.4 0.4]}, ...
                "Mass", 10, "Inertia", [1 0.2 1], "Collisions", false, "LinearVelocity", v0);
            phx.BushingJoint(cart, host, "PointA", offset, "PointB", [0 0 0], ...
                "LinearLimits", 0, ...
                "AngularStiffness", [3000 0 5000], "AngularDamping", [200 0 200], ...
                "AngularLimits", [-0.6 0.6; 0 0; -0.6 0.6], ...
                "Visible", false);
        end
        for side = [1 -1]
            % Compliant treads (polyurethane): 1 mm under a quarter of the
            % car, damped at a ratio of 0.3
            pWheel = [xw side*gauge/2 -0.05];
            wheel = phx.Body(ax, "Position", pCart + pWheel*R0', "Orientation", R0, ...
                "Shape", wheelShape{(3 - side)/2}, "Friction", 0.3, ...
                "ContactStiffness", 1e6, "ContactDamping", 6e3, ...
                "LinearVelocity", v0, "AngularVelocity", speed0/r*trk.L(i0, :));
            phx.RevoluteJoint(host, wheel, "PointA", pWheel - offset, "PointB", [0 0 0], ...
                "AxisA", [0 1 0], "AxisB", [0 1 0], "Visible", false);
        end
    end
end

function trk = coasterTrack(p)
% Centerline of the track sampled every 2 cm, with its frame: tangent T, lateral L
% (to the left) and up U, the normal of the rail plane.
%
% Every element is shaped by a curvature profile of the form sin^2, which
% starts and ends at zero, so the curvature never jumps between elements.
% The drop and the loop lie in a vertical plane, the loop shifted sideways
% so that its exit passes beside its entry. The turn is level and banked
% at atan(v^2 k/g), with v^2 from the height lost, reduced by BankFactor
% for the losses on the way.
    arguments
        p.Height = 14            % crest above the ground (m)
        p.Base = 1.2             % height of the low parts (m)
        p.DropLength = 26        % horizontal length of the drop (m)
        p.LoopLength = 44        % arc length of the loop (m)
        p.LoopShift = 3          % sideways shift across the loop (m)
        p.TurnLength = 56        % arc length of the 180 deg turn (m)
        p.Speed = 3              % speed on the crest (m/s)
        p.BankFactor = 0.8       % expected share of the loss-free v^2
    end
    g = 9.81;
    ds = 0.02;
    straight = (ds:ds:4)';

    % Crest long enough for the train, cosine-shaped drop and a straight
    x = (-5:ds:3)';
    P = [x, 0*x, p.Height + 0*x];
    x = (ds:ds/4:p.DropLength)';
    P = [P; P(end, 1) + x, 0*x, p.Base + (p.Height - p.Base)*(1 + cos(pi*x/p.DropLength))/2];
    P = [P; P(end, 1) + straight, 0*straight, p.Base + 0*straight];

    % Loop: pitch angle 2*pi*s/S - sin(2*pi*s/S), curvature (4*pi/S)*sin^2(pi*s/S)
    S = p.LoopLength;
    s = (0:ds/4:S)';
    th = 2*pi*s/S - sin(2*pi*s/S);
    loop = [cumtrapz(s, cos(th)), p.LoopShift*(s/S - sin(2*pi*s/S)/(2*pi)), cumtrapz(s, sin(th))];
    P = [P; P(end, :) + loop(2:end, :)];
    P = [P; P(end, 1) + straight, P(end, 2) + 0*straight, p.Base + 0*straight];
    nVertical = size(P, 1);

    % Level turn to the left: heading pi*(s/S - sin(2*pi*s/S)/(2*pi))
    S = p.TurnLength;
    s = (0:ds/4:S)';
    psi = pi*(s/S - sin(2*pi*s/S)/(2*pi));
    turn = [cumtrapz(s, cos(psi)), cumtrapz(s, sin(psi)), 0*s];
    kappa = (2*pi/S)*sin(pi*s(2:end)/S).^2;
    P = [P; P(end, :) + turn(2:end, :)];

    % Run-out, ending in a kicker that rises 0.8 m over its last 6 m
    x = (ds:ds:20)';
    P = [P; P(end, 1) - x, P(end, 2) + 0*x, p.Base + 0.8*max(x - 14, 0).^2/36];
    kappa = [zeros(nVertical, 1); kappa; 0*x];

    % Resample by arc length
    d = [0; cumsum(vecnorm(diff(P), 2, 2))];
    trk.s = (0:ds:d(end))';
    P = interp1(d, P, trk.s, "pchip");
    kappa = interp1(d, kappa, trk.s);
    vertical = trk.s <= d(nVertical);

    T = gradient(P')';
    T = T./vecnorm(T, 2, 2);
    % Drop and loop: the lateral axis stays (nearly) along +y
    Uv = cross(T, repmat([0 1 0], size(T, 1), 1), 2);
    Uv = Uv./vecnorm(Uv, 2, 2);
    Lv = cross(Uv, T, 2);
    % Turn and run-out: level rail plane, banked towards the centre of the turn
    Lh = cross(repmat([0 0 1], size(T, 1), 1), T, 2);
    Lh = Lh./vecnorm(Lh, 2, 2);
    Uh = cross(T, Lh, 2);
    bank = atan(p.BankFactor*(p.Speed^2 + 2*g*(p.Height - P(:, 3))).*kappa/g);
    Ub = cos(bank).*Uh + sin(bank).*Lh;
    Lb = cos(bank).*Lh - sin(bank).*Uh;

    trk.P = P;
    trk.T = T;
    trk.U = Ub;
    trk.U(vertical, :) = Uv(vertical, :);
    trk.L = Lb;
    trk.L(vertical, :) = Lv(vertical, :);
end

function [V, F] = railMesh(trk, offset, radius)
% Tube around the rail at the lateral offset from the centerline, with a
% ring wherever the rail has turned by 0.5 deg or run 0.12 m. Each ring is
% turned by half a segment, so a flat facet faces up for the wheels. The
% wheels feel every bend of the polyline as a small knock, so a coarser
% step makes the ride rougher; much finer triangles disturb the contacts.
    C = trk.P + offset*trk.L;
    T = gradient(C')';
    T = T./vecnorm(T, 2, 2);
    ring = false(size(trk.s));
    ring([1 end]) = true;
    last = 1;
    for i = 2:numel(trk.s)
        if dot(T(i, :), T(last, :)) <= cosd(0.5) || trk.s(i) - trk.s(last) >= 0.12
            ring(i) = true;
            last = i;
        end
    end
    [V, F] = tube(C(ring, :), trk.U(ring, :), radius, 16, 0.5);
end

function [V, F] = trackStructure(trk, gauge)
% Spine tube under the centre, ties across to the rails and columns down
% to the ground, all as one mesh
    step = round(1/(trk.s(2) - trk.s(1)));      % a sample every metre
    [V, F] = tube(trk.P(1:5:end, :) - 0.35*trk.U(1:5:end, :), trk.U(1:5:end, :), 0.1, 10, 0);

    for i = 1:step:numel(trk.s)
        % Tie: a flat bar from rail to rail under the spine
        c = trk.P(i, :) - 0.2*trk.U(i, :);
        [Vt, Ft] = tube(c + [-1; 1]*(gauge/2*trk.L(i, :)), [1; 1]*trk.T(i, :), 0.04, 4, 0.5);
        [V, F] = appendMesh(V, F, Vt, Ft);

        % Column every 4 m where the track faces up
        if mod(i - 1, 4*step) == 0 && trk.U(i, 3) > 0.3
            base = trk.P(i, :) - 0.35*trk.U(i, :);
            [Vc, Fc] = tube([base; base(1:2) 0], [1 0 0; 1 0 0], 0.12, 10, 0);
            [V, F] = appendMesh(V, F, Vc, Fc);
        end
    end
end

function [V, F] = tube(C, U, radius, nSeg, phase)
% Open tube of nSeg facets along the polyline C; U orients the rings
    T = gradient(C')';
    if size(C, 1) == 2
        T = repmat(C(2, :) - C(1, :), 2, 1);
    end
    T = T./vecnorm(T, 2, 2);
    N1 = U - dot(U, T, 2).*T;
    N1 = N1./vecnorm(N1, 2, 2);
    N2 = cross(T, N1, 2);
    a = 2*pi*((0:nSeg - 1) + phase)/nSeg;
    n = size(C, 1);
    V = zeros(n*nSeg, 3);
    for k = 1:nSeg
        V(k:nSeg:end, :) = C + radius*(cos(a(k))*N1 + sin(a(k))*N2);
    end
    [I, K] = ndgrid(0:n - 2, 0:nSeg - 1);
    v00 = I(:)*nSeg + K(:) + 1;
    v01 = I(:)*nSeg + mod(K(:) + 1, nSeg) + 1;
    F = [v00, v01, v01 + nSeg; v00, v01 + nSeg, v00 + nSeg];
end

function [V, F] = appendMesh(V, F, V2, F2)
% Merge a second mesh into the first
    F = [F; F2 + size(V, 1)];
    V = [V; V2];
end
