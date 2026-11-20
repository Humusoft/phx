function phxex_plow(Options)
% PHXEX_PLOW Spring tine plowing granular soil in a rotating soil bin
%
% A ring trough filled with granular soil turns under a vertical share
% carried by a spring tine, a flexible beam clamped to a shank. The share is
% lowered into the soil in steps. The joint that holds the clamp measures
% the load on the mount: the inset plots the draft, the force against the
% direction of travel, and the bending moment (solid) and torsion (dotted)
% the tine puts on the mount. The tine bends back under the draft and is
% coloured by its von Mises stress. The camera moves in close before the
% last plunge.
%
% Options:
%     Depths        - working depths of the share below the soil surface (m)
%     Speed         - soil speed at the share (m/s)
%     Revolutions   - revolutions of the bin held at each depth
%     YoungsModulus - Young's modulus of the tine (Pa)
%     Angle         - turn of the whole tool about the shank axis (deg); a
%                     share set at an angle to the travel also loads the
%                     mount with a side force
%     Offset        - sideways shift of the share from the tine axis (m),
%                     along its width; the draft then twists the tine. The
%                     tine end has to stay well inside the share, or it is
%                     welded only partly and the tine breaks up.
%
% Examples:
%     phxex_plow(Depths = [0.02 0.04 0.06 0.08], Revolutions = 1)
%     phxex_plow(Angle = 0, Offset = 0)    % share square on the tine axis
%     phxex_plow(Angle = 30)

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Options.Depths (1, :) double {mustBePositive} = [0.03 0.06]
        Options.Speed (1, 1) double {mustBePositive} = 0.6
        Options.Revolutions (1, 1) double {mustBePositive} = 0.6
        Options.YoungsModulus (1, 1) double {mustBePositive} = 5e8
        Options.Angle (1, 1) double {mustBeInRange(Options.Angle, -60, 60)} = 10
        Options.Offset (1, 1) double {mustBeInRange(Options.Offset, -0.025, 0.025)} = 0.02
    end

    % Bin: ring trough between radii Ri and Ro, the share runs at radius Rm
    Ri = 0.325;
    Ro = 0.575;
    Rm = 0.45;
    wall = 0.18;
    t = 0.03;

    % Soil: 22 mm grains
    d = 0.022;

    % Share: vertical steel plate, width x thickness x height
    share = [0.1 0.025 0.15];

    % Tine: free length L, width b, thickness h in the direction of travel;
    % its ends are set e deep into the clamp and the share
    L = 0.3;
    b = 0.04;
    h = 0.02;
    e = 0.04;

    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "Texture", "tiles", ...
        "DefaultCameraPosition", [1.35 -1.35 1.25], "DefaultCameraTarget", [0 0 0.35]);

    % Trough revolved about z, standing on the floor; the closed profile
    % starts at axis coordinate 0
    profile = [0 Ri; 0 Ro; wall Ro; wall Ro+t; -t Ro+t; -t Ri-t; wall Ri-t; wall Ri; 0 Ri];
    bin = phx.Body(ax, "Type", "kinematic", "Position", [0 0 t], ...
        "Shape", phx.shape.Revolution("Axis", "z", "Profile", profile, "Segments", 90, ...
        "Envelope", "concave", "Color", 0.35, "Texture", "xor", "TextureBlend", 0.5));

    % Soil poured in as loose layers
    rng(1);
    s = 1.25*d;
    [x, y, z] = ndgrid(-Ro:s:Ro, -Ro:s:Ro, t + d/2 + 0.002 + (0:8)*s);
    r = hypot(x, y);
    in = r > Ri + d/2 + 0.003 & r < Ro - d/2 - 0.003;
    P = [x(in) y(in) z(in)];
    P(:, 1:2) = P(:, 1:2) + 0.22*d*(rand(size(P, 1), 2) - 0.5);

    % The loose layers stand higher than the walls; an invisible collar on
    % top of the walls keeps them in until they have settled
    collar = 0.18;
    collars = [ ...
        phx.Body(ax, "Type", "static", "Visible", false, "Position", [0 0 t + wall + collar/2], ...
            "Shape", {"Tube", "InnerDiameter", 2*Ro, "WallThickness", t, "Height", collar, ...
            "Segments", 90, "Envelope", "concave"}), ...
        phx.Body(ax, "Type", "static", "Visible", false, "Position", [0 0 t + wall + collar/2], ...
            "Shape", {"Tube", "InnerDiameter", 2*(Ri - t), "WallThickness", t, "Height", collar, ...
            "Segments", 90, "Envelope", "concave"})];
    soil = phx.Particles(ax, "Positions", P, "Radius", d/2, "Density", 2650, "Friction", 0.25, ...
        "Color", [0.75 0.55 0.35], "RandomTint", 0.4, "MarkerSize", 28);

    % Tool, top down: a kinematic shank sliding in a guide on a cantilever
    % arm, a clamp held to it by the joint that measures the mount load, the
    % tine and the share. All of it starts above the soil as it is poured in.
    offset = 0.25 + L + share(3);   % shank centre above the bottom of the share
    zStart = t + 0.3 + offset;
    zTop = zStart - 0.13;
    turn = makehgtform("zrotate", deg2rad(Options.Angle));
    turn = turn(1:3, 1:3);
    shank = phx.Body(ax, "Type", "kinematic", "Collisions", false, "Position", [Rm 0 zStart], ...
        "Orientation", turn, "Shape", {"Box", "Size", [0.03 0.03 0.4], "Color", [0.8 0.8 0.8]});

    % The tine is solved after the rigid bodies and meets the joint once per
    % step, so the clamp is given a large mass and inertia: the step then
    % cannot turn or shift it noticeably, and it holds the tine end rigidly
    clamp = phx.Body(ax, "Collisions", false, "Position", [Rm 0 zStart - 0.225], "Orientation", turn, ...
        "Shape", {"Box", "Size", [b + 0.02 h + 0.02 0.05], "Color", [0.3 0.3 0.3]}, ...
        "Mass", 5, "Inertia", [5 5 5]);
    holder = phx.FixedJoint(shank, clamp, "TransformA", makehgtform("translate", [0 0 -0.225]));

    tine = phx.FlexibleBody(ax, "Shape", {"Box", "Size", [b h L + 2*e], "ElementResolution", 1}, ...
        "Position", [Rm 0 zStart - 0.25 - L/2], "Orientation", turn, ...
        "Density", 1800, "YoungsModulus", Options.YoungsModulus, "PoissonsRatio", 0.3, "Damping", 2, ...
        "Colormap", turbo(64), "ColorRange", [0 10e6]);
    shift = (turn*[Options.Offset; 0; 0])';
    plate = phx.Body(ax, "Position", [Rm 0 zStart - offset + share(3)/2] + shift, "Orientation", turn, ...
        "Shape", {"Box", "Size", share, "Density", 7800, "Color", [0.25 0.35 0.6]});
    phx.IntersectionWeld(tine, clamp);
    phx.IntersectionWeld(tine, plate);

    % Column in the middle of the ring, with the arm out to the shank
    arm = phx.Body(ax, "Type", "static", "Collisions", false, "Position", [Rm/2 0 zTop], ...
        "Shape", {"Box", "Size", [Rm + 0.1 0.05 0.05], "Color", [0.45 0.45 0.5]});
    post = phx.Body(ax, "Type", "static", "Collisions", false, "Position", [0 0 zTop/2], ...
        "Shape", {"Cylinder", "Diameter", 0.08, "Height", zTop + 0.05, "Color", [0.45 0.45 0.5]});

    % Schedule: settle, run in above the soil, then a ramp to each depth and
    % a hold of the given number of revolutions
    settle = 1.5;
    omega = Options.Speed/Rm;
    holdTime = Options.Revolutions*2*pi/omega;
    ramp = 1;
    ts = settle + 1 + [0, cumsum(repmat([ramp holdTime], 1, numel(Options.Depths)))];
    zs = [0.02, reshape([-Options.Depths; -Options.Depths], 1, [])];
    tEnd = ts(end);
    phx.Script(bin, {"EulerAngles", [0 settle tEnd]', [0 0 0; 0 0 0; 0 0 omega*(tEnd - settle)]});

    % Inset plot of the draft and the mount moments, 0.1 s means, in the
    % top left corner whatever the size of the window
    inset = uiaxes(viewer.Figure, "Units", "normalized", "Position", [0.005 0.68 0.32 0.31], ...
        "XLim", [0 tEnd], "Box", "on");
    inset.XAxis.Color = [0.2 0.2 0.2];      % the title and the time axis stand on the light sky
    inset.Title.Color = [0.2 0.2 0.2];
    grid(inset, "on");
    xlabel(inset, "time (s)");
    title(inset, "Share above the soil");
    yyaxis(inset, "left");
    ylabel(inset, "draft (N)");
    p.Line = plot(inset, NaN, NaN, "LineWidth", 1.5);
    yyaxis(inset, "right");
    ylabel(inset, "bending, torsion (N m)");
    p.LineM = plot(inset, NaN, NaN, "-", NaN, NaN, ":", "LineWidth", 1);

    % Shank driver and load recorder, called every substep
    p.Shank = shank;
    p.Clamp = clamp;
    p.Plate = plate;
    p.Holder = holder;
    p.Offset = offset;
    p.Schedule = [settle NaN; ts' zs'];   % share bottom above the surface (m)
    p.Start = zStart - offset;
    p.Surface = NaN;                % set once the soil has settled
    p.Weight = 9.81*(clamp.Mass + tine.Mass + plate.Mass);
    p.Sum = zeros(1, 6);
    p.Count = 0;
    p.Time = NaN(1, ceil(tEnd/0.1) + 1);
    % draft and side force (N), bending moment and torsion at the mount
    % (N m), share deflection and working depth (m)
    p.Load = NaN(numel(p.Time), 6);
    p.Half = share(3)/2;
    p.Shift = shift(2);
    p.Inset = inset;
    driver = phx.Function({shank, clamp, plate}, @driveShank);
    driver.UserData = p;

    opt = phx.engine.Settings("Threads", 8, "LengthScale", 0.025);
    sim = phx.Simulation({bin, collars(1), collars(2), soil, shank, clamp, plate, tine, arm, post}, ...
        "EngineSettings", opt);
    dt = 0.005;

    % Let the soil settle, open the collar and measure the surface along the
    % path of the share
    sim.step(settle, round(settle/dt));
    [collars.Collisions] = deal(false);
    P = soil.Positions;
    near = abs(hypot(P(:, 1), P(:, 2)) - Rm) < 0.05;
    p = driver.UserData;
    p.Surface = prctile(P(near, 3), 98) + d/2;
    p.Schedule(1, 2) = p.Start - p.Surface;
    driver.UserData = p;

    % Run up to the last plunge, then move the camera in close to the tine
    tClose = ts(end - 2);
    sim.step(tClose - settle, round((tClose - settle)/dt));
    viewer.CameraPosition = [1.2 -0.5 0.75];
    viewer.CameraTarget = [0.4 0 0.31];
    sim.step(tEnd - tClose, round((tEnd - tClose)/dt));

    % Mean loads at each depth, skipping the first second of each hold
    p = driver.UserData;
    fprintf("depth (mm)          force (N)          mount moment (N m)   deflection\n");
    fprintf("  set  working   draft    side      bending   torsion        (mm)\n");
    for k = 1:numel(Options.Depths)
        held = p.Time > ts(2*k) + 1 & p.Time <= ts(2*k + 1);
        m = mean(p.Load(held, :), 1);
        fprintf("%5.0f %8.0f %7.1f %7.1f %12.1f %9.2f %11.1f\n", 1000*Options.Depths(k), 1000*m(6), ...
            m(1), m(2), m(3), m(4), 1000*m(5));
    end
    delete(sim);
end

function driveShank(fcn, ~, dt, time)
% Set the shank height from the schedule and accumulate the loads on the
% mount; the soil moves along +y at the share
    p = fcn.UserData;
    if isnan(p.Surface)
        return
    end
    s = p.Schedule;
    zb = interp1(s(:, 1), s(:, 2), min(max(time, s(1, 1)), s(end, 1)));
    pos = p.Shank.Position;
    p.Shank.Position = [pos(1:2) p.Surface + zb + p.Offset];

    % The joint holds the clamp against the soil force and the weight of the
    % tool below it: the draft along y, a side force along x (radial), the
    % horizontal moment bends the tine, the vertical one twists it
    force = -p.Holder.ForceB + [0 0 p.Weight];
    moment = -p.Holder.TorqueB;
    share = p.Plate.Position;
    deflection = share(2) - p.Clamp.Position(2) - p.Shift;
    depth = p.Surface - (share(3) - p.Half);
    p.Sum = p.Sum + [force(2) force(1) hypot(moment(1), moment(2)) moment(3) deflection depth];
    p.Count = p.Count + 1;
    if p.Count*dt >= 0.1 - dt/2
        k = find(isnan(p.Time), 1);
        p.Time(k) = time;
        p.Load(k, :) = p.Sum/p.Count;
        p.Sum(:) = 0;
        p.Count = 0;
        set(p.Line, "XData", p.Time(1:k), "YData", p.Load(1:k, 1));
        set(p.LineM(1), "XData", p.Time(1:k), "YData", p.Load(1:k, 3));
        set(p.LineM(2), "XData", p.Time(1:k), "YData", p.Load(1:k, 4));
        if zb < 0
            title(p.Inset, sprintf("Share %.0f mm deep, deflected %.0f mm", 1000*depth, 1000*deflection));
        end
    end
    fcn.UserData = p;
end
