function phxex_gyrostab(duration, spinRate, maxAmplitude)
% PHXEX_GYROSTAB Gyroscopic roll stabilizer - who keeps the cargo in growing seas?
%
% Two identical floating platforms ride the same beam waves side by side,
% each loaded with four cargo crates on the corners of its deck, held only
% by friction. The left platform is bare; the right one carries a passive
% gyroscopic stabilizer of the kind used on ships: a fast-spinning steel
% disk whose shaft is gimballed to the platform by two revolute joints.
% The disk gets only an initial angular velocity and is never driven
% afterwards.
%
% When a wave rolls the platform, the roll rate crosses the disk's angular
% momentum and the gyroscopic torque swings the gimbal; the reaction of
% that precession opposes the roll. Everything emerges from rigid-body
% dynamics alone, without a single line of control code.
%
% The sea builds up: the wave amplitude ramps from calm to maxAmplitude
% over the run. Once the roll of the bare platform exceeds the friction
% angle of its crates they slide overboard one after another, while the
% stabilized platform keeps rolling gently and keeps its cargo. The demo
% reports the roll of both platforms and the fate of all eight crates.
%
% Input Arguments:
%     duration     - simulated time in seconds (default 30)
%     spinRate     - initial disk angular velocity in rad/s (default 150)
%     maxAmplitude - wave amplitude reached at the end of the run (default 0.1)
%
% Example:
%     phxex_gyrostab            % default run
%     phxex_gyrostab(30, 0)     % disk parked -> both platforms lose cargo

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        duration (1, 1) double {mustBePositive} = 30
        spinRate (1, 1) double {mustBeNonnegative} = 150
        maxAmplitude (1, 1) double {mustBePositive} = 0.1
    end

    platSize = [2 2 0.4];       % platform, density 500 -> floats half draft
    gyroZ = 0.5;                % gimbal center above the platform center
    diskR = 0.4;
    offset = 1.5;               % platform distance from the scene center

    % Cargo: four crates on the deck corners, same relative positions and
    % friction on both platforms (clear of the spinning disk in the middle)
    crateSize = 0.4;
    crateMu = 0.15;
    cratePos = [-0.65 -0.65; -0.65 0.65; 0.65 0.65; 0.65 -0.65];

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.3], ...
        "DefaultCameraPosition", [1 -7 3]);

    % Two identical platforms; the waves travel along y, so both see the
    % same wave phase and roll about their x axis
    platA = phx.Body(ax, "Position", [-offset 0 0], ...
        "Shape", {"Box", "Size", platSize, "Density", 500, ...
        "Color", [1 1 1]}, "Friction", [0.5 0 0]);
    platB = phx.Body(ax, "Position", [offset 0 0], ...
        "Shape", {"Box", "Size", platSize, "Density", 500, ...
        "Color", [1 1 1], "SkeletPoints", [0 -0.6 gyroZ; 0 0.6 gyroZ]}, "Friction", [0.5 0 0]);
    plats = [platA platB];
    platName = ["bare", "gyro"];

    crates = phx.Body.empty;
    for p = 1:2
        for i = 1:4
            crates(p, i) = phx.Body(ax, ...
                "Position", plats(p).Position + [cratePos(i, :), platSize(3)/2 + crateSize/2], ...
                "Shape", {"Box", "Size", crateSize*[1 1 1], "Density", 400, ...
                "Color", [0.85 0.7 0.5], "Texture", resdir+"woodtile.jpg", "TextureBlend", 0.5}, ...
                "Friction", [crateMu 0 0]);
        end
    end

    % Gyro stabilizer on platform B: shaft hinged to the platform about
    % the transverse y axis (gimbal), spinning disk hinged to the shaft
    % about its z axis; neither part collides with anything
    shaft = phx.Body(ax, "Position", [offset 0 gyroZ], ...
        "Shape", {"Cylinder", "Axis", "z", "Radius", 0.04, "Height", 0.55, ...
        "Density", 2000, "Color", [0.4 0.4 0.4], "SkeletPoints", [0 -0.6 0; 0 0.6 0], "SkeletStyle", "line"}, ...
        "Collisions", false);
    disk = phx.Body(ax, "Position", [offset 0 gyroZ], ...
        "Shape", {"Cylinder", "Radius", diskR, "Height", 0.06, ...
        "Density", 7800, "Color", [0.95 0.55 0.5], "Texture", resdir+"checker2.png", "TextureBlend", 0.5});

    gimbal = phx.RevoluteJoint(platB, shaft, "PointA", [0 0 gyroZ], ...
        "PointB", [0 0 0], "AxisA", [0 1 0], "AxisB", [0 1 0]);
    phx.RevoluteJoint(shaft, disk, "PointA", [0 0 0], "PointB", [0 0 0]);

    % The flywheel is only spun up initially, never driven again
    disk.AngularVelocity = [0 0 spinRate];

    % Beam sea along y with a linearly growing amplitude; fallen crates
    % float, so they get the same buoyancy as the platforms
    kw = 4*pi/6;
    ww = sqrt(9.81*kw);
    rampRate = maxAmplitude/duration;
    amp = @(t) min(rampRate*t, maxAmplitude);
    wave = @(x, y, t) amp(t).*sin(kw*y - ww*t);
    phx.Buoyancy([plats crates(:)'], "LevelFunction", wave, ...
        "LinearDamping", 400, "AngularDamping", 20, ...
        "SurfaceSize", [8 8], "SurfaceStep", 0.25);

    % Sleeping must stay disabled: the stabilizer works purely through
    % the spinning disk, which must never be deactivated
    sim = phx.Simulation(ax);

    % Fine time step: the fast-spinning disk inside a joint chain needs it
    dt = 0.002;
    subSteps = 25;

    fallen = false(2, 4);
    fallTime = nan(2, 4);
    hist = struct("t", [], "rollA", [], "rollB", [], "gimbal", []);

    while sim.Time < duration
        sim.step(dt*subSteps, subSteps, subSteps);
        t = sim.Time;

        ea = platA.EulerAngles;
        eb = platB.EulerAngles;
        hist.t(end + 1) = t;
        hist.rollA(end + 1) = ea(1);
        hist.rollB(end + 1) = eb(1);
        hist.gimbal(end + 1) = gimbal.Angle;

        % A crate has departed once it leaves the deck area in the frame
        % of its own platform (slid over an edge or dropped below deck)
        for p = 1:2
            for i = find(~fallen(p, :))
                rel = plats(p).Transform\[crates(p, i).Position 1]';
                if max(abs(rel(1:2))) > platSize(2)/2 + crateSize || rel(3) < 0
                    fallen(p, i) = true;
                    fallTime(p, i) = t;
                    fprintf("Crate lost from the %s platform at t = %.1f s (wave amplitude %.2f m).\n", ...
                        platName(p), t, amp(t));
                end
            end
        end

        viewer.displayText(sprintf(...
            "t = %4.1f s   amplitude %.2f m   roll: bare %+5.1f deg / gyro %+5.1f deg   cargo: %d/4 vs %d/4", ...
            t, amp(t), rad2deg(ea(1)), rad2deg(eb(1)), ...
            4 - nnz(fallen(1, :)), 4 - nnz(fallen(2, :))));
        pause(0);
    end
    delete(sim);

    % Summary: cargo kept and the roll comparison
    fprintf("\nCargo kept: bare platform %d/4, gyro platform %d/4.\n", ...
        4 - nnz(fallen(1, :)), 4 - nnz(fallen(2, :)));
    rmsA = rms(hist.rollA);
    rmsB = rms(hist.rollB);
    fprintf("Roll over the whole run: bare %.2f deg RMS, gyro %.2f deg RMS (%.0f %% reduction).\n", ...
        rad2deg(rmsA), rad2deg(rmsB), 100*(1 - rmsB/rmsA));

    % Roll history of both platforms and the gimbal precession
    figure(2);
    clf;
    subplot(2, 1, 1);
    plot(hist.t, rad2deg(hist.rollA), hist.t, rad2deg(hist.rollB), "LineWidth", 1.2);
    hold on
    for i = find(fallen(1, :))
        xline(fallTime(1, i), "-k", "lost", "LabelVerticalAlignment", "bottom");
    end
    for i = find(fallen(2, :))
        xline(fallTime(2, i), ":k", "lost (gyro)", "LabelVerticalAlignment", "bottom");
    end
    grid on; ylabel("roll [deg]");
    legend("bare platform", "gyro platform");
    title(sprintf("Growing seas: bare platform loses its cargo (spin %.0f rad/s)", spinRate));
    subplot(2, 1, 2);
    plot(hist.t, rad2deg(hist.gimbal), "LineWidth", 1.2);
    grid on; xlabel("time [s]"); ylabel("gimbal angle [deg]");
    title("Gimbal precession absorbing the wave roll moment");

end
