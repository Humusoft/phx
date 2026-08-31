function phxex_eulerdisk(duration, tilt, rollFriction)
% PHXEX_EULERDISK Euler's disk - the same launch for three mass distributions
%
% Three steel bodies with the same rim diameter and the same thickness are
% launched side by side in the state of the classic Euler's disk toy: tilted
% out of the horizontal and rolling on the rim without slipping, so that the
% contact point runs around the rim while the body itself stays put. Only
% the mass distribution differs: a ring (a cylinder with a hole, revolved
% with Envelope "cylinder" so that its collision shape stays a primitive
% cylinder and the hole changes the inertia and not the contact), a solid
% disk and a flat cone standing on its base.
%
% A body rolling on its rim precesses at Omega = sqrt(m*g*R/(A*sin(a))) for
% mass m, transverse inertia A, rim radius R and tilt a, which for a thin
% disk (A = m*R^2/4) is the familiar sqrt(4*g/(R*sin(a))). The launch is the
% steady state of the solid disk, so the disk precesses smoothly while the
% ring, over-spun for its mass distribution, and the cone, under-spun,
% answer with nutation. Rolling resistance drains all three: the tilt
% collapses and the precession rate, the rattle of the real toy, runs away
% as it does so.
%
% A phx.Logger records the pose and the spin of all three bodies; the tilt
% and the precession rate in the plots are computed from those afterwards.
%
% Input Arguments:
%     duration     - simulated time in seconds (default 20)
%     tilt         - initial tilt of all three bodies in degrees (default 25)
%     rollFriction - rolling resistance of the rim on the table (default 0.001)
%
% Example:
%     phxex_eulerdisk               % all three settle flat within some 17 s
%     phxex_eulerdisk(10, 25, 0)    % no dissipation, nothing settles
%     phxex_eulerdisk(30, 45)       % steeper start, longer collapse

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        duration (1, 1) double {mustBePositive} = 20
        tilt (1, 1) double {mustBeInRange(tilt, 5, 80)} = 25
        rollFriction (1, 1) double {mustBeNonnegative} = 0.001
    end

    rimR = 0.15;
    bodyH = 0.04;
    holeR = 0.7*rimR;
    spacing = 0.4;              % the nutating ring wanders a few cm
    a0 = deg2rad(tilt);

    % Center height that puts the rim of a body on the table
    centerZ = rimR*sin(a0) + bodyH/2*cos(a0);

    % Figure setup - the camera stands back to hold the whole row
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0 -1.5 0.6]);

    % Table: sliding friction only, all dissipation belongs to the rims
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.02], ...
        "Shape", {"Box", "Size", [1.5 1.5 0.04], "Texture", "tiles"}, "Friction", [1 0 0]);

    % Closed annulus cross-section of the ring, rows of [axial radial]
    annulus = [-1 holeR; -1 rimR; 1 rimR; 1 holeR; -1 holeR].*[bodyH/2 1];

    % The three bodies: same rim, same thickness, same tilt about the x axis
    ring = phx.Body(ax, "Position", [-spacing 0 centerZ], "EulerAngles", [a0 0 0], ...
        "Shape", {"Revolution", "Profile", annulus, "Envelope", "cylinder", ...
        "Style", "flat", "Density", 7800, "Color", [0.4 0.65 0.35], ...
        "Texture", "checker", "TextureBlend", 0.3}, "Friction", [1 rollFriction 0]);
    disk = phx.Body(ax, "Position", [0 0 centerZ], "EulerAngles", [a0 0 0], ...
        "Shape", {"Cylinder", "Radius", rimR, "Height", bodyH, ...
        "Density", 7800, "Color", [0.85 0.55 0.2], "Texture", "checker", ...
        "TextureBlend", 0.3}, "Friction", [1 rollFriction 0]);
    cone = phx.Body(ax, "Position", [spacing 0 centerZ], "EulerAngles", [a0 0 0], ...
        "Shape", {"Cone", "Radius", rimR, "Height", bodyH, ...
        "Density", 7800, "Color", [0.3 0.5 0.8], "Texture", "checker", ...
        "TextureBlend", 0.3}, "Friction", [1 rollFriction 0]);
    bodies = [ring disk cone];
    names = ["ring", "cylinder", "cone"];
    nBody = numel(bodies);

    % Add planar shadow
    phx.PlanarShadow(bodies);

    % Rolling without slipping about a fixed center: the angular velocity
    % points from the center to the contact point, with the magnitude the
    % solid disk needs for steady precession. All three get that same launch.
    steadyRate = sqrt(disk.Mass*9.81*rimR./(disk.Inertia(1)*sin(a0)));
    omega = steadyRate*sin(a0)*[0 cos(a0) sin(a0)];
    set(bodies, "AngularVelocity", omega);

    % Trail of one rim point of each body
    for k = 1:nBody
        phx.Trace(bodies(k), "Point", [0 -rimR -bodyH/2], "TracePoints", 300, "Overlay", true);
    end

    % Pose and spin of every body, enough for both plots
    logger = phx.Logger(bodies, "Parameters", ["Orientation", "AngularVelocity"], ...
        "Frequency", 100);

    % A rim contact at a grazing angle needs a small step and no margin
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.BulletSettings("Margin", 0));
    dt = 0.0005;
    subSteps = 20;

    while sim.Time < duration
        sim.step(dt*subSteps, subSteps, subSteps);

        % Done once every body lies flat and has stopped rattling
        pose = vertcat(bodies.Position);
        if all(pose(:, 3) < 0.55*bodyH) && ...
                all(vecnorm(vertcat(bodies.AngularVelocity), 2, 2) < 0.5)
            break
        end
    end
    delete(sim);

    % Tilt of a body axis off the vertical and the rate at which the azimuth
    % of that axis turns, both from the recorded pose and spin
    t = logger.Time;
    tiltDeg = zeros(numel(t), nBody);
    precRate = zeros(numel(t), nBody);
    for k = 1:nBody
        R = logger.getChannel(2*k - 1);
        n = R(:, 7:9);                          % body axis, third column of R
        lean = vecnorm(n(:, 1:2), 2, 2);
        dn = cross(logger.getChannel(2*k), n, 2);
        tiltDeg(:, k) = atan2d(lean, n(:, 3));
        precRate(:, k) = abs(n(:, 1).*dn(:, 2) - n(:, 2).*dn(:, 1))./max(lean.^2, eps);
    end

    % Results. The azimuth of an upright axis is ill defined, so the
    % precession rate is plotted only while a body is still leaning.
    clf(figure(2));
    rolling = tiltDeg > 1;
    precRate(~rolling) = NaN;

    subplot(2, 1, 1);
    plot(t, tiltDeg, "LineWidth", 1.2);
    grid on; xlabel("time [s]"); ylabel("tilt [deg]");
    legend(names, "Location", "northeast");
    title("Same launch, three mass distributions");

    subplot(2, 1, 2);
    plot(t, precRate, "LineWidth", 1.2);
    ylim([0 1.1*max(precRate(rolling))]);
    grid on; xlabel("time [s]"); ylabel("precession [rad/s]");
    title("The rattle runs away as the tilt collapses");

end